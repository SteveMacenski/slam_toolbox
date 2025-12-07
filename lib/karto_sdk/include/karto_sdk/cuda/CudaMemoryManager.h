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

#ifndef KARTO_SDK_CUDA_MEMORY_MANAGER_H_
#define KARTO_SDK_CUDA_MEMORY_MANAGER_H_

#ifdef SLAM_TOOLBOX_CUDA_ENABLED

#include <cstddef>
#include <cstdint>

namespace karto {
namespace cuda {

/**
 * @brief Manages CUDA memory allocation with support for unified memory
 *
 * Optimized for Jetson platforms where CPU and GPU share physical memory,
 * enabling zero-copy data access between host and device.
 */
class CudaMemoryManager {
public:
    CudaMemoryManager();
    ~CudaMemoryManager();

    /**
     * @brief Initialize CUDA context and query device capabilities
     * @return true if initialization succeeded
     */
    bool initialize();

    /**
     * @brief Check if manager is initialized
     */
    bool isInitialized() const { return m_initialized; }

    /**
     * @brief Check if unified memory is supported (Jetson/integrated GPU)
     */
    bool hasUnifiedMemory() const { return m_hasUnifiedMemory; }

    /**
     * @brief Pre-allocated buffers for scan matching operations
     */
    struct ScanMatchBuffers {
        // Output arrays
        double* responses;          // Response values for each pose
        double* poseX;              // X coordinate for each pose
        double* poseY;              // Y coordinate for each pose
        double* poseTheta;          // Heading for each pose

        // Input arrays (copied per scan match)
        double* xPoses;             // X search positions
        double* yPoses;             // Y search positions
        int32_t* gridIndices;       // Pre-computed grid indices for (x,y) positions

        // Lookup table (flattened)
        int32_t* lookupOffsets;     // Flattened angle lookup arrays
        int32_t* lookupSizes;       // Number of points per angle
        int32_t* lookupStarts;      // Start index for each angle

        // Reduction buffers
        double* blockMax;           // Per-block maximum values
        int32_t* blockMaxIdx;       // Per-block maximum indices

        // Capacity tracking
        size_t maxPoses;            // Maximum poses allocated
        size_t maxLookupSize;       // Maximum lookup table size
        size_t maxAngles;           // Maximum angles allocated
    };

    /**
     * @brief Get or allocate buffers for scan matching
     * @param requiredPoses Number of pose hypotheses
     * @param requiredLookupSize Total size of flattened lookup arrays
     * @param requiredAngles Number of angle bins
     * @return Reference to buffer struct
     */
    ScanMatchBuffers& getBuffers(size_t requiredPoses,
                                  size_t requiredLookupSize,
                                  size_t requiredAngles);

    /**
     * @brief Allocate unified memory (zero-copy on Jetson)
     * @param size Size in bytes
     * @return Pointer to allocated memory
     */
    void* allocateUnified(size_t size);

    /**
     * @brief Free unified memory
     */
    void freeUnified(void* ptr);

    /**
     * @brief Allocate device memory
     */
    void* allocateDevice(size_t size);

    /**
     * @brief Free device memory
     */
    void freeDevice(void* ptr);

    /**
     * @brief Synchronize device
     */
    void synchronize();

    /**
     * @brief Get CUDA stream for async operations
     */
    void* getStream() { return m_stream; }

private:
    void freeBuffers();
    void allocateBuffers(size_t poses, size_t lookupSize, size_t angles);

    ScanMatchBuffers m_buffers;
    bool m_initialized;
    bool m_hasUnifiedMemory;
    int m_deviceId;
    void* m_stream;
};

}  // namespace cuda
}  // namespace karto

#endif  // SLAM_TOOLBOX_CUDA_ENABLED
#endif  // KARTO_SDK_CUDA_MEMORY_MANAGER_H_
