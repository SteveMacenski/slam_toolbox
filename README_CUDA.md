# CUDA Accelerated Scan Matching for SLAM Toolbox

This document describes the optional CUDA acceleration feature for scan matching in SLAM Toolbox, designed primarily for NVIDIA Jetson platforms.

## Motivation

Scan matching is the computational bottleneck in 2D SLAM. For each incoming laser scan, the algorithm must evaluate thousands of pose hypotheses to find the best alignment between the new scan and the existing map. This involves:

- Evaluating ~20,000-100,000 pose hypotheses per scan match
- Each hypothesis requires ~100-400 grid lookups
- Total: ~2-9 million memory accesses per scan

The existing TBB (Threading Building Blocks) implementation parallelizes this across CPU cores, but embedded platforms like the Jetson Orin Nano have limited CPU performance compared to their GPU capabilities.

## Goals

1. **Reduce scan matching latency** - Enable real-time SLAM on embedded GPU platforms
2. **Leverage Jetson hardware** - Utilize the 1024 CUDA cores on Jetson Orin Nano
3. **Maintain compatibility** - Keep TBB as the default, CUDA is opt-in
4. **Zero-copy on Jetson** - Use unified memory to avoid CPU-GPU transfer overhead

## Target Hardware

This implementation is optimized for:

- **NVIDIA Jetson Orin Nano** (8GB)
  - CUDA Compute Capability 8.7 (Ampere)
  - 1024 CUDA cores
  - Unified memory architecture (CPU/GPU share physical memory)

Other supported architectures (via CMake):
- SM 8.6 (RTX 30 series, Jetson AGX Orin)
- SM 7.5 (Turing)
- SM 7.0 (Volta)

## How It Works

### CPU Implementation (TBB)

The original scan matching parallelizes over Y positions:

```
for each Y position (parallel via TBB):
    for each X position:
        for each angle:
            compute correlation response
```

This creates ~30-100 parallel tasks.

### CUDA Implementation

The GPU version parallelizes over all dimensions simultaneously:

```
for each (Y, X, angle) triple (one CUDA thread each):
    compute correlation response
```

This creates ~20,000-100,000 parallel threads, matching the GPU's massive parallelism.

### Key Optimizations

1. **Unified Memory**: On Jetson, CPU and GPU share physical memory. We use `cudaMallocManaged` with memory advise hints to enable zero-copy access.

2. **Pre-computed Lookups**: Grid index offsets are pre-computed for each angle and stored in GPU memory.

3. **Coalesced Access**: Thread indexing is designed to maximize memory coalescing.

4. **Persistent Buffers**: GPU memory is allocated once and reused across scan matches.

## Building with CUDA Support

### Prerequisites

- NVIDIA CUDA Toolkit 11.0 or later
- CMake 3.18 or later (for modern CUDA support)
- On Jetson: JetPack 5.0 or later

### Build Commands

```bash
# Standard build (no CUDA, uses TBB)
colcon build --packages-select slam_toolbox

# Build with CUDA support
colcon build --packages-select slam_toolbox \
    --cmake-args -DSLAM_TOOLBOX_USE_CUDA=ON

# For specific CUDA architectures (optional)
colcon build --packages-select slam_toolbox \
    --cmake-args -DSLAM_TOOLBOX_USE_CUDA=ON \
                 -DCMAKE_CUDA_ARCHITECTURES=87
```

### Verify CUDA Build

When built with CUDA support, you'll see:
```
-- CUDA acceleration enabled for scan matching
```

## Configuration

### YAML Parameter

Add to your config file (e.g., `mapper_params_online_sync.yaml`):

```yaml
slam_toolbox:
  ros__parameters:
    use_cuda: true  # Enable CUDA acceleration
    # ... other parameters
```

### Launch File Override

```bash
ros2 launch slam_toolbox online_sync_launch.py use_cuda:=true
```

### Runtime Behavior

- If `use_cuda: true` but CUDA initialization fails, it automatically falls back to TBB
- If built without CUDA support, the parameter is ignored (with a warning)
- Both sequential and loop closure scan matchers use the same setting

## File Structure

```
lib/karto_sdk/
├── include/karto_sdk/cuda/
│   ├── ScanMatcherCuda.h         # CUDA scan matcher interface
│   └── CudaMemoryManager.h       # Unified memory management
├── src/cuda/
│   ├── ScanMatcherCuda.cu        # CUDA kernel implementations
│   └── CudaMemoryManager.cu      # Memory allocation utilities
├── src/
│   └── Mapper.cpp                # Modified: CUDA dispatch logic
└── CMakeLists.txt                # Modified: Optional CUDA compilation
```

## Expected Performance

| Platform | TBB (CPU) | CUDA (GPU) | Speedup |
|----------|-----------|------------|---------|
| Jetson Orin Nano | ~15-30ms | ~2-5ms | 5-10x |
| Desktop (RTX 3080) | ~5-10ms | ~0.5-1ms | 10-20x |

*Actual performance depends on scan density, search space size, and system load.*

## Troubleshooting

### CUDA Not Detected

```
CUDA support not compiled, using TBB
```

Rebuild with `-DSLAM_TOOLBOX_USE_CUDA=ON`.

### CUDA Initialization Failed

```
CUDA scan matcher initialization failed, falling back to TBB
```

Check:
- NVIDIA driver is installed (`nvidia-smi`)
- CUDA runtime is available (`nvcc --version`)
- Sufficient GPU memory

### Performance Not Improved

On systems with fast CPUs and slow GPUs, CUDA may not provide benefits. The overhead of kernel launch and synchronization can exceed the parallel speedup for small search spaces.

Consider adjusting:
- `correlation_search_space_dimension` - Larger values benefit more from GPU
- `loop_search_space_dimension` - Loop closure benefits significantly

## Technical Details

### Memory Layout

| Buffer | Type | Size | Purpose |
|--------|------|------|---------|
| responses | double | nX × nY × nAngles | Correlation scores |
| poseX/Y/Theta | double | nX × nY × nAngles | Output poses |
| lookupOffsets | int32 | nAngles × nPoints | Pre-computed grid offsets |
| gridIndices | int32 | nX × nY | Grid position for each (x,y) |

### Kernel Configuration

- Block size: 256 threads (optimized for Ampere)
- Grid size: ceil(totalPoses / 256) blocks
- Shared memory: Not used (data fits in L2 cache)

### Synchronization

The implementation uses synchronous kernel execution to maintain compatibility with the existing scan matching flow. Future optimizations could pipeline multiple scans using CUDA streams.

## Contributing

When modifying the CUDA implementation:

1. Test on both CUDA and non-CUDA builds
2. Verify numerical equivalence with TBB implementation
3. Profile on target hardware (Jetson) before optimizing
4. Maintain the fallback path to TBB

## References

- [SLAM Toolbox Paper](https://joss.theoj.org/papers/10.21105/joss.02783)
- [NVIDIA Jetson Orin Nano](https://developer.nvidia.com/embedded/jetson-orin-nano)
- [CUDA Programming Guide](https://docs.nvidia.com/cuda/cuda-c-programming-guide/)
