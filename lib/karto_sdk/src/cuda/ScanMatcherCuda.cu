/*
 * Copyright 2024
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "karto_sdk/cuda/ScanMatcherCuda.h"
#include "karto_sdk/Mapper.h"

#include <cuda_runtime.h>
#include <cmath>
#include <iostream>
#include <limits>

namespace karto {
namespace cuda {

// Constants matching CPU implementation
constexpr int32_t INVALID_SCAN = std::numeric_limits<int32_t>::max();
constexpr double GRID_STATES_OCCUPIED = 100.0;
constexpr double DISTANCE_PENALTY_GAIN = 0.5;
constexpr double ANGLE_PENALTY_GAIN = 0.5;

// Kernel block size (optimized for Ampere architecture)
constexpr int BLOCK_SIZE = 256;

/**
 * @brief Main correlation kernel - evaluates all pose hypotheses in parallel
 *
 * Each thread handles one (x, y, angle) position triple.
 * Memory layout: responses[y * nX * nAngles + x * nAngles + angle]
 */
__global__ void CorrelateScanKernel(
    const uint8_t* __restrict__ gridData,
    int32_t gridDataSize,
    int32_t gridWidthStep,
    const int32_t* __restrict__ lookupOffsets,
    const int32_t* __restrict__ lookupSizes,
    const int32_t* __restrict__ lookupStarts,
    const double* __restrict__ xPoses,
    const double* __restrict__ yPoses,
    const int32_t* __restrict__ gridIndices,
    double searchCenterX,
    double searchCenterY,
    double searchCenterHeading,
    double searchAngleOffset,
    double searchAngleResolution,
    int nX,
    int nY,
    int nAngles,
    bool doPenalize,
    double distanceVariancePenalty,
    double angleVariancePenalty,
    double minDistancePenalty,
    double minAnglePenalty,
    double* __restrict__ responses,
    double* __restrict__ outPoseX,
    double* __restrict__ outPoseY,
    double* __restrict__ outPoseTheta)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int totalPoses = nX * nY * nAngles;

    if (idx >= totalPoses) return;

    // Decode (y, x, angle) from linear index
    // Layout: angle varies fastest, then x, then y
    int angleIndex = idx % nAngles;
    int xyIdx = idx / nAngles;
    int xIdx = xyIdx % nX;
    int yIdx = xyIdx / nX;

    // Get position offsets
    double x = xPoses[xIdx];
    double y = yPoses[yIdx];
    double newPositionX = searchCenterX + x;
    double newPositionY = searchCenterY + y;

    // Get grid index for this (x, y) position
    int xyLinearIdx = yIdx * nX + xIdx;
    int32_t gridPositionIndex = gridIndices[xyLinearIdx];

    // Compute angle
    double startAngle = searchCenterHeading - searchAngleOffset;
    double angle = startAngle + angleIndex * searchAngleResolution;

    // Get lookup array for this angle
    int32_t lookupStart = lookupStarts[angleIndex];
    int32_t lookupSize = lookupSizes[angleIndex];

    // Compute response (sum of grid values at lookup positions)
    double response = 0.0;

    if (lookupSize > 0 && gridPositionIndex >= 0) {
        for (int i = 0; i < lookupSize; i++) {
            int32_t offset = lookupOffsets[lookupStart + i];

            // Skip invalid scan points
            if (offset == INVALID_SCAN) continue;

            int32_t pointGridIndex = gridPositionIndex + offset;

            // Bounds check
            if (pointGridIndex >= 0 && pointGridIndex < gridDataSize) {
                response += static_cast<double>(gridData[pointGridIndex]);
            }
        }

        // Normalize response
        response /= (lookupSize * GRID_STATES_OCCUPIED);
    }

    // Apply penalty if enabled
    if (doPenalize && response > 0.0) {
        double squaredDistance = x * x + y * y;
        double distancePenalty = 1.0 - (DISTANCE_PENALTY_GAIN * squaredDistance / distanceVariancePenalty);
        distancePenalty = fmax(distancePenalty, minDistancePenalty);

        double angleDiff = angle - searchCenterHeading;
        double squaredAngleDistance = angleDiff * angleDiff;
        double anglePenalty = 1.0 - (ANGLE_PENALTY_GAIN * squaredAngleDistance / angleVariancePenalty);
        anglePenalty = fmax(anglePenalty, minAnglePenalty);

        response *= (distancePenalty * anglePenalty);
    }

    // Store results
    responses[idx] = response;
    outPoseX[idx] = newPositionX;
    outPoseY[idx] = newPositionY;
    outPoseTheta[idx] = angle;
}

/**
 * @brief Reduction kernel to find maximum response
 *
 * Uses shared memory for efficient parallel reduction within blocks.
 */
__global__ void FindMaxResponseKernel(
    const double* __restrict__ responses,
    int n,
    double* __restrict__ blockMax,
    int32_t* __restrict__ blockMaxIdx)
{
    extern __shared__ char sharedMem[];
    double* sdata = reinterpret_cast<double*>(sharedMem);
    int32_t* sidx = reinterpret_cast<int32_t*>(sharedMem + blockDim.x * sizeof(double));

    int tid = threadIdx.x;
    int i = blockIdx.x * blockDim.x + threadIdx.x;

    // Load data into shared memory
    if (i < n) {
        sdata[tid] = responses[i];
        sidx[tid] = i;
    } else {
        sdata[tid] = -1.0;
        sidx[tid] = -1;
    }
    __syncthreads();

    // Reduction in shared memory
    for (int s = blockDim.x / 2; s > 0; s >>= 1) {
        if (tid < s) {
            if (sdata[tid + s] > sdata[tid]) {
                sdata[tid] = sdata[tid + s];
                sidx[tid] = sidx[tid + s];
            }
        }
        __syncthreads();
    }

    // Write block result
    if (tid == 0) {
        blockMax[blockIdx.x] = sdata[0];
        blockMaxIdx[blockIdx.x] = sidx[0];
    }
}

// ============================================================================
// ScanMatcherCuda implementation
// ============================================================================

ScanMatcherCuda::ScanMatcherCuda()
    : m_initialized(false)
    , m_cachedNAngles(0)
    , m_cachedLookupSize(0)
{
}

ScanMatcherCuda::~ScanMatcherCuda()
{
}

bool ScanMatcherCuda::initialize()
{
    if (m_initialized) {
        return true;
    }

    m_memManager = std::make_unique<CudaMemoryManager>();
    if (!m_memManager->initialize()) {
        std::cerr << "Failed to initialize CUDA memory manager" << std::endl;
        return false;
    }

    m_initialized = true;
    std::cout << "CUDA scan matcher initialized successfully" << std::endl;
    return true;
}

void ScanMatcherCuda::prepareLookupArrays(
    const GridIndexLookup<kt_int8u>* gridLookup,
    kt_int32u nAngles)
{
    // Calculate total lookup size
    size_t totalLookupSize = 0;
    for (kt_int32u i = 0; i < nAngles; i++) {
        const LookupArray* pOffsets = gridLookup->GetLookupArray(i);
        if (pOffsets) {
            totalLookupSize += pOffsets->GetSize();
        }
    }

    // Get buffers
    auto& buffers = m_memManager->getBuffers(0, totalLookupSize, nAngles);

    // Copy lookup data
    size_t offset = 0;
    for (kt_int32u i = 0; i < nAngles; i++) {
        const LookupArray* pOffsets = gridLookup->GetLookupArray(i);
        kt_int32u size = pOffsets ? pOffsets->GetSize() : 0;

        buffers.lookupStarts[i] = static_cast<int32_t>(offset);
        buffers.lookupSizes[i] = static_cast<int32_t>(size);

        if (size > 0 && pOffsets) {
            const kt_int32s* srcPtr = pOffsets->GetArrayPointer();
            std::memcpy(&buffers.lookupOffsets[offset], srcPtr, size * sizeof(int32_t));
            offset += size;
        }
    }

    m_cachedNAngles = nAngles;
    m_cachedLookupSize = totalLookupSize;
}

void ScanMatcherCuda::computeGridIndices(
    const std::vector<kt_double>& xPoses,
    const std::vector<kt_double>& yPoses,
    const Pose2& searchCenter,
    kt_int32s gridWidth,
    kt_int32s gridWidthStep,
    const Vector2<kt_int32s>& startGridPoint)
{
    size_t nX = xPoses.size();
    size_t nY = yPoses.size();
    size_t totalXY = nX * nY;

    auto& buffers = m_memManager->getBuffers(totalXY * 100, m_cachedLookupSize, m_cachedNAngles);

    // Copy pose arrays
    std::memcpy(buffers.xPoses, xPoses.data(), nX * sizeof(double));
    std::memcpy(buffers.yPoses, yPoses.data(), nY * sizeof(double));

    // Pre-compute grid indices for all (x, y) combinations
    // This mirrors the WorldToGrid + GridIndex computation in the CPU code
    for (size_t yIdx = 0; yIdx < nY; yIdx++) {
        for (size_t xIdx = 0; xIdx < nX; xIdx++) {
            // Grid point is startGridPoint offset by search position indices
            int32_t gridX = startGridPoint.GetX() + static_cast<int32_t>(xIdx);
            int32_t gridY = startGridPoint.GetY() + static_cast<int32_t>(yIdx);

            // Compute linear grid index (row-major)
            int32_t gridIndex = gridX + gridY * gridWidthStep;

            buffers.gridIndices[yIdx * nX + xIdx] = gridIndex;
        }
    }
}

void ScanMatcherCuda::correlateScanParallel(
    const kt_int8u* gridData,
    kt_int32s gridDataSize,
    kt_int32s gridWidth,
    kt_int32s gridWidthStep,
    const std::vector<kt_double>& xPoses,
    const std::vector<kt_double>& yPoses,
    kt_int32u nAngles,
    const Pose2& searchCenter,
    kt_double searchAngleOffset,
    kt_double searchAngleResolution,
    kt_bool doPenalize,
    kt_double distanceVariancePenalty,
    kt_double angleVariancePenalty,
    kt_double minDistancePenalty,
    kt_double minAnglePenalty,
    const GridIndexLookup<kt_int8u>* gridLookup,
    const Vector2<kt_int32s>& startGridPoint,
    std::pair<kt_double, Pose2>* poseResponses)
{
    if (!m_initialized) {
        std::cerr << "CUDA scan matcher not initialized" << std::endl;
        return;
    }

    int nX = static_cast<int>(xPoses.size());
    int nY = static_cast<int>(yPoses.size());
    int totalPoses = nX * nY * static_cast<int>(nAngles);

    if (totalPoses == 0) return;

    // Prepare lookup arrays on GPU
    prepareLookupArrays(gridLookup, nAngles);

    // Compute grid indices for all positions
    computeGridIndices(xPoses, yPoses, searchCenter, gridWidth, gridWidthStep, startGridPoint);

    // Get buffers (now properly sized)
    auto& buffers = m_memManager->getBuffers(totalPoses, m_cachedLookupSize, nAngles);

    // Get CUDA stream
    cudaStream_t stream = static_cast<cudaStream_t>(m_memManager->getStream());

    // Launch correlation kernel
    int numBlocks = (totalPoses + BLOCK_SIZE - 1) / BLOCK_SIZE;

    CorrelateScanKernel<<<numBlocks, BLOCK_SIZE, 0, stream>>>(
        gridData,
        gridDataSize,
        gridWidthStep,
        buffers.lookupOffsets,
        buffers.lookupSizes,
        buffers.lookupStarts,
        buffers.xPoses,
        buffers.yPoses,
        buffers.gridIndices,
        searchCenter.GetX(),
        searchCenter.GetY(),
        searchCenter.GetHeading(),
        searchAngleOffset,
        searchAngleResolution,
        nX, nY, static_cast<int>(nAngles),
        doPenalize,
        distanceVariancePenalty,
        angleVariancePenalty,
        minDistancePenalty,
        minAnglePenalty,
        buffers.responses,
        buffers.poseX,
        buffers.poseY,
        buffers.poseTheta);

    // Synchronize to ensure kernel completes
    m_memManager->synchronize();

    // Copy results back to host format
    for (int i = 0; i < totalPoses; i++) {
        double response = buffers.responses[i];
        double x = buffers.poseX[i];
        double y = buffers.poseY[i];
        double theta = buffers.poseTheta[i];

        // Normalize angle to [-pi, pi]
        while (theta > M_PI) theta -= 2.0 * M_PI;
        while (theta < -M_PI) theta += 2.0 * M_PI;

        poseResponses[i] = std::make_pair(response, Pose2(x, y, theta));
    }
}

kt_int32u ScanMatcherCuda::findBestResponse(
    const std::pair<kt_double, Pose2>* poseResponses,
    kt_int32u poseResponseSize,
    kt_double& bestResponse,
    Pose2& bestPose)
{
    // For now, use CPU reduction (unified memory makes this efficient)
    // TODO: Implement GPU reduction for larger response arrays

    bestResponse = 0.0;
    kt_int32u bestIndex = 0;
    kt_int32u validCount = 0;

    for (kt_int32u i = 0; i < poseResponseSize; i++) {
        double response = poseResponses[i].first;

        if (response > 0.0) {
            validCount++;
        }

        if (response > bestResponse) {
            bestResponse = response;
            bestIndex = i;
        }
    }

    if (bestResponse > 0.0) {
        bestPose = poseResponses[bestIndex].second;
    }

    return validCount;
}

}  // namespace cuda
}  // namespace karto
