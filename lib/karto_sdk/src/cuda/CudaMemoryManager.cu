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

#include "karto_sdk/cuda/CudaMemoryManager.h"

#include <cuda_runtime.h>
#include <cstring>
#include <iostream>

namespace karto {
namespace cuda {

namespace {

// Helper macro for CUDA error checking
#define CUDA_CHECK(call) \
    do { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            std::cerr << "CUDA error in " << __FILE__ << ":" << __LINE__ \
                      << ": " << cudaGetErrorString(err) << std::endl; \
            return false; \
        } \
    } while (0)

#define CUDA_CHECK_VOID(call) \
    do { \
        cudaError_t err = call; \
        if (err != cudaSuccess) { \
            std::cerr << "CUDA error in " << __FILE__ << ":" << __LINE__ \
                      << ": " << cudaGetErrorString(err) << std::endl; \
        } \
    } while (0)

}  // anonymous namespace

CudaMemoryManager::CudaMemoryManager()
    : m_initialized(false)
    , m_hasUnifiedMemory(false)
    , m_deviceId(0)
    , m_stream(nullptr)
{
    std::memset(&m_buffers, 0, sizeof(m_buffers));
}

CudaMemoryManager::~CudaMemoryManager()
{
    if (m_initialized) {
        freeBuffers();

        if (m_stream) {
            cudaStreamDestroy(static_cast<cudaStream_t>(m_stream));
        }
    }
}

bool CudaMemoryManager::initialize()
{
    if (m_initialized) {
        return true;
    }

    // Get device count
    int deviceCount = 0;
    CUDA_CHECK(cudaGetDeviceCount(&deviceCount));

    if (deviceCount == 0) {
        std::cerr << "No CUDA devices found" << std::endl;
        return false;
    }

    // Use device 0 (can be made configurable)
    m_deviceId = 0;
    CUDA_CHECK(cudaSetDevice(m_deviceId));

    // Query device properties
    cudaDeviceProp props;
    CUDA_CHECK(cudaGetDeviceProperties(&props, m_deviceId));

    std::cout << "CUDA device: " << props.name << std::endl;
    std::cout << "  Compute capability: " << props.major << "." << props.minor << std::endl;
    std::cout << "  Total memory: " << (props.totalGlobalMem / (1024 * 1024)) << " MB" << std::endl;
    std::cout << "  Multiprocessors: " << props.multiProcessorCount << std::endl;

    // Check for unified memory support (integrated GPU like Jetson)
    m_hasUnifiedMemory = (props.integrated != 0);
    if (m_hasUnifiedMemory) {
        std::cout << "  Unified memory: supported (integrated GPU)" << std::endl;
    }

    // Create CUDA stream for async operations
    cudaStream_t stream;
    CUDA_CHECK(cudaStreamCreate(&stream));
    m_stream = static_cast<void*>(stream);

    m_initialized = true;
    return true;
}

void CudaMemoryManager::freeBuffers()
{
    if (m_buffers.responses) freeUnified(m_buffers.responses);
    if (m_buffers.poseX) freeUnified(m_buffers.poseX);
    if (m_buffers.poseY) freeUnified(m_buffers.poseY);
    if (m_buffers.poseTheta) freeUnified(m_buffers.poseTheta);
    if (m_buffers.xPoses) freeUnified(m_buffers.xPoses);
    if (m_buffers.yPoses) freeUnified(m_buffers.yPoses);
    if (m_buffers.gridIndices) freeUnified(m_buffers.gridIndices);
    if (m_buffers.lookupOffsets) freeUnified(m_buffers.lookupOffsets);
    if (m_buffers.lookupSizes) freeUnified(m_buffers.lookupSizes);
    if (m_buffers.lookupStarts) freeUnified(m_buffers.lookupStarts);
    if (m_buffers.blockMax) freeUnified(m_buffers.blockMax);
    if (m_buffers.blockMaxIdx) freeUnified(m_buffers.blockMaxIdx);

    std::memset(&m_buffers, 0, sizeof(m_buffers));
}

void CudaMemoryManager::allocateBuffers(size_t poses, size_t lookupSize, size_t angles)
{
    // Free existing buffers if they're too small
    if (m_buffers.maxPoses < poses ||
        m_buffers.maxLookupSize < lookupSize ||
        m_buffers.maxAngles < angles) {
        freeBuffers();
    } else {
        // Existing buffers are sufficient
        return;
    }

    // Add some headroom to avoid frequent reallocations
    size_t allocPoses = static_cast<size_t>(poses * 1.5);
    size_t allocLookup = static_cast<size_t>(lookupSize * 1.5);
    size_t allocAngles = static_cast<size_t>(angles * 1.5);

    // Allocate output arrays
    m_buffers.responses = static_cast<double*>(allocateUnified(allocPoses * sizeof(double)));
    m_buffers.poseX = static_cast<double*>(allocateUnified(allocPoses * sizeof(double)));
    m_buffers.poseY = static_cast<double*>(allocateUnified(allocPoses * sizeof(double)));
    m_buffers.poseTheta = static_cast<double*>(allocateUnified(allocPoses * sizeof(double)));

    // Allocate input arrays (sized for max X/Y dimensions)
    size_t maxDim = static_cast<size_t>(std::sqrt(static_cast<double>(allocPoses))) + 1;
    m_buffers.xPoses = static_cast<double*>(allocateUnified(maxDim * sizeof(double)));
    m_buffers.yPoses = static_cast<double*>(allocateUnified(maxDim * sizeof(double)));
    m_buffers.gridIndices = static_cast<int32_t*>(allocateUnified(allocPoses * sizeof(int32_t)));

    // Allocate lookup arrays
    m_buffers.lookupOffsets = static_cast<int32_t*>(allocateUnified(allocLookup * sizeof(int32_t)));
    m_buffers.lookupSizes = static_cast<int32_t*>(allocateUnified(allocAngles * sizeof(int32_t)));
    m_buffers.lookupStarts = static_cast<int32_t*>(allocateUnified(allocAngles * sizeof(int32_t)));

    // Allocate reduction buffers (one per block, max 1024 blocks)
    size_t maxBlocks = (allocPoses + 255) / 256;
    if (maxBlocks > 1024) maxBlocks = 1024;
    m_buffers.blockMax = static_cast<double*>(allocateUnified(maxBlocks * sizeof(double)));
    m_buffers.blockMaxIdx = static_cast<int32_t*>(allocateUnified(maxBlocks * sizeof(int32_t)));

    m_buffers.maxPoses = allocPoses;
    m_buffers.maxLookupSize = allocLookup;
    m_buffers.maxAngles = allocAngles;
}

CudaMemoryManager::ScanMatchBuffers& CudaMemoryManager::getBuffers(
    size_t requiredPoses,
    size_t requiredLookupSize,
    size_t requiredAngles)
{
    allocateBuffers(requiredPoses, requiredLookupSize, requiredAngles);
    return m_buffers;
}

void* CudaMemoryManager::allocateUnified(size_t size)
{
    void* ptr = nullptr;
    cudaError_t err = cudaMallocManaged(&ptr, size, cudaMemAttachGlobal);

    if (err != cudaSuccess) {
        std::cerr << "cudaMallocManaged failed: " << cudaGetErrorString(err) << std::endl;
        return nullptr;
    }

    // On Jetson (integrated GPU), advise the runtime about memory access patterns
    if (m_hasUnifiedMemory) {
        // Prefer CPU location but allow GPU access (zero-copy)
        cudaMemAdvise(ptr, size, cudaMemAdviseSetPreferredLocation, cudaCpuDeviceId);
        cudaMemAdvise(ptr, size, cudaMemAdviseSetAccessedBy, m_deviceId);
    }

    return ptr;
}

void CudaMemoryManager::freeUnified(void* ptr)
{
    if (ptr) {
        CUDA_CHECK_VOID(cudaFree(ptr));
    }
}

void* CudaMemoryManager::allocateDevice(size_t size)
{
    void* ptr = nullptr;
    cudaError_t err = cudaMalloc(&ptr, size);

    if (err != cudaSuccess) {
        std::cerr << "cudaMalloc failed: " << cudaGetErrorString(err) << std::endl;
        return nullptr;
    }

    return ptr;
}

void CudaMemoryManager::freeDevice(void* ptr)
{
    if (ptr) {
        CUDA_CHECK_VOID(cudaFree(ptr));
    }
}

void CudaMemoryManager::synchronize()
{
    if (m_stream) {
        cudaStreamSynchronize(static_cast<cudaStream_t>(m_stream));
    } else {
        cudaDeviceSynchronize();
    }
}

}  // namespace cuda
}  // namespace karto
