cmake_minimum_required(VERSION 3.5)
project(karto_sdk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-backtrace-limit=0")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

# Optional CUDA support for accelerated scan matching
option(SLAM_TOOLBOX_USE_CUDA "Enable CUDA acceleration for scan matching" OFF)

if(SLAM_TOOLBOX_USE_CUDA)
  find_package(CUDAToolkit REQUIRED)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 17)
  # Jetson Orin Nano (SM 8.7) and common desktop GPUs
  set(CMAKE_CUDA_ARCHITECTURES 87 86 75 70)
  add_definitions(-DSLAM_TOOLBOX_CUDA_ENABLED)
  message(STATUS "CUDA acceleration enabled for scan matching")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system serialization filesystem thread)
find_package(TBB REQUIRED NO_CMAKE_PACKAGE_REGISTRY)

set(dependencies
  rclcpp
  rclcpp_lifecycle
)

include_directories(include ${EIGEN3_INCLUDE_DIRS} 
                            ${Boost_INCLUDE_DIR}
                            ${BOOST_INCLUDE_DIRS}
                            ${TBB_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

include_directories(include ${EIGEN3_INCLUDE_DIRS} ${TBB_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Build library with optional CUDA sources
set(KARTO_SOURCES src/Karto.cpp src/Mapper.cpp)

if(SLAM_TOOLBOX_USE_CUDA)
  list(APPEND KARTO_SOURCES
    src/cuda/ScanMatcherCuda.cu
    src/cuda/CudaMemoryManager.cu
  )
endif()

add_library(kartoSlamToolbox SHARED ${KARTO_SOURCES})

target_link_libraries(kartoSlamToolbox PUBLIC
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)
target_link_libraries(kartoSlamToolbox PUBLIC ${Boost_LIBRARIES} TBB::tbb)

if(SLAM_TOOLBOX_USE_CUDA)
  target_link_libraries(kartoSlamToolbox PUBLIC CUDA::cudart)
  target_include_directories(kartoSlamToolbox PUBLIC ${CUDAToolkit_INCLUDE_DIRS})
endif()

install(DIRECTORY include/
	DESTINATION include/
)
