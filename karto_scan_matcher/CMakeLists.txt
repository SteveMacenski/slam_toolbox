cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_boost_directories()
find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_PATH})
link_libraries(${PYTHON_LIBRARIES})

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})


rosbuild_add_library(karto_scan_matcher src/karto_scan_matcher.cpp)
target_link_libraries(karto_scan_matcher OpenKarto)

rosbuild_add_library(karto_scan_matching src/python_exports.cpp)
set_target_properties(karto_scan_matching
  PROPERTIES
  PREFIX "")
target_link_libraries(karto_scan_matching karto_scan_matcher)
rosbuild_link_boost(karto_scan_matching python)


rosbuild_add_executable(bin/test_scan_matcher src/test_scan_matcher.cpp)
target_link_libraries(bin/test_scan_matcher karto_scan_matcher)

# rosbuild_add_gtest(bin/scan_match_test test/scan_match_test.cpp)
# target_link_libraries(bin/scan_match_test karto_scan_matcher)