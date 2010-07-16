cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init(karto)

set(ROS_BUILD_TYPE Release)

# Build Karto
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.karto
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of Karto failed")
endif(_make_failed)

# Build the ROS wrapper
include_directories(${PROJECT_SOURCE_DIR}/karto/include)
link_directories(${PROJECT_SOURCE_DIR}/karto/lib)
rosbuild_add_executable(bin/slam_karto src/slam_karto.cpp src/spa_solver.cpp)
target_link_libraries(bin/slam_karto OpenKarto)
