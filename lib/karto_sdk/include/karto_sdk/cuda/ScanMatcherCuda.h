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

#ifndef KARTO_SDK_CUDA_SCAN_MATCHER_CUDA_H_
#define KARTO_SDK_CUDA_SCAN_MATCHER_CUDA_H_

#ifdef SLAM_TOOLBOX_CUDA_ENABLED

#include <memory>
#include <vector>
#include <utility>

#include "karto_sdk/Karto.h"
#include "karto_sdk/cuda/CudaMemoryManager.h"

namespace karto {

// Forward declarations
template<typename T> class GridIndexLookup;

namespace cuda {

/**
 * @brief CUDA-accelerated scan matcher for 2D SLAM
 *
 * Replaces TBB parallelization with GPU kernels for the scan correlation
 * algorithm. Each thread evaluates one (x, y, angle) pose hypothesis.
 *
 * Optimized for Jetson Orin Nano (SM 8.7, 8GB unified memory).
 */
class ScanMatcherCuda {
public:
    ScanMatcherCuda();
    ~ScanMatcherCuda();

    /**
     * @brief Initialize CUDA resources
     * @return true if initialization succeeded, false otherwise
     */
    bool initialize();

    /**
     * @brief Check if CUDA matcher is ready
     */
    bool isInitialized() const { return m_initialized; }

    /**
     * @brief Execute parallel scan correlation on GPU
     *
     * This replaces the TBB parallel_for_each loop in CorrelateScan().
     * All pose hypotheses are evaluated in parallel on the GPU.
     *
     * @param gridData Pointer to correlation grid data (uint8)
     * @param gridDataSize Total size of grid data array
     * @param gridWidth Grid width in cells
     * @param gridWidthStep Grid row stride (aligned width)
     * @param xPoses X position offsets to evaluate
     * @param yPoses Y position offsets to evaluate
     * @param nAngles Number of angle bins
     * @param searchCenter Center pose for the search
     * @param searchAngleOffset Angular search half-width (radians)
     * @param searchAngleResolution Angular step size (radians)
     * @param doPenalize Whether to apply distance/angle penalties
     * @param distanceVariancePenalty Distance penalty parameter
     * @param angleVariancePenalty Angle penalty parameter
     * @param minDistancePenalty Minimum distance penalty
     * @param minAnglePenalty Minimum angle penalty
     * @param gridLookup Pre-computed grid lookup arrays
     * @param startGridPoint Grid coordinates of search origin
     * @param poseResponses Output array of (response, pose) pairs
     */
    void correlateScanParallel(
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
        std::pair<kt_double, Pose2>* poseResponses);

    /**
     * @brief Find best response and compute mean pose using GPU reduction
     *
     * @param poseResponses Array of (response, pose) pairs
     * @param poseResponseSize Number of poses to search
     * @param bestResponse Output: best response value found
     * @param bestPose Output: pose with best response
     * @return Number of poses with response > 0
     */
    kt_int32u findBestResponse(
        const std::pair<kt_double, Pose2>* poseResponses,
        kt_int32u poseResponseSize,
        kt_double& bestResponse,
        Pose2& bestPose);

private:
    /**
     * @brief Copy lookup arrays to GPU memory
     */
    void prepareLookupArrays(const GridIndexLookup<kt_int8u>* gridLookup,
                              kt_int32u nAngles);

    /**
     * @brief Compute grid indices for all (x,y) positions
     */
    void computeGridIndices(
        const std::vector<kt_double>& xPoses,
        const std::vector<kt_double>& yPoses,
        const Pose2& searchCenter,
        kt_int32s gridWidth,
        kt_int32s gridWidthStep,
        const Vector2<kt_int32s>& startGridPoint);

    std::unique_ptr<CudaMemoryManager> m_memManager;
    bool m_initialized;

    // Cached lookup table info
    kt_int32u m_cachedNAngles;
    size_t m_cachedLookupSize;
};

}  // namespace cuda
}  // namespace karto

#endif  // SLAM_TOOLBOX_CUDA_ENABLED
#endif  // KARTO_SDK_CUDA_SCAN_MATCHER_CUDA_H_
