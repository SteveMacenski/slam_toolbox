cmake_minimum_required(VERSION 3.5)
project(slam_toolbox)

add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/CMake/")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/karto_sdk/cmake)
set(CMAKE_CXX_FLAGS "-fpermissive -std=c++17")

#karto_sdk lib
set(BUILD_SHARED_LIBS ON)
add_subdirectory(lib/karto_sdk)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
    karto_sdk
    rosconsole
    roscpp
    sensor_msgs
    sparse_bundle_adjustment
    tf2
    tf2_ros
    visualization_msgs
    pluginlib
    tf2_geometry_msgs
    interactive_markers
    rviz
    map_server
    slam_toolbox_msgs
)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CSparse REQUIRED)
find_package(G2O REQUIRED)
find_package(Cholmod REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)
find_package(rostest REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)
find_package(Boost REQUIRED system serialization filesystem thread)

include_directories(include ${catkin_INCLUDE_DIRS} 
                            ${EIGEN3_INCLUDE_DIRS} 
                            ${CHOLMOD_INCLUDE_DIR}
                            ${Boost_INCLUDE_DIR}
                            ${BOOST_INCLUDE_DIRS}
                            ${TBB_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS
      include
      ${BOOST_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
      ${TBB_INCLUDE_DIRS}
    LIBRARIES
      ceres_solver_plugin
      toolbox_lib
      slam_toolbox_rviz_plugin
    CATKIN_DEPENDS
      message_filters
      nav_msgs
      rosconsole
      roscpp
      sparse_bundle_adjustment
      sensor_msgs
      rviz
      tf2
      tf2_ros
      visualization_msgs
      pluginlib
      message_runtime
      tf2_geometry_msgs
      interactive_markers
      slam_toolbox_msgs
    DEPENDS
      EIGEN3
      Boost
)

set(CMAKE_AUTOMOC ON)
if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt4_wrap_cpp(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt5_wrap_cpp(${ARGN})
  endmacro()
endif()

#### Ceres Plugin
add_library(ceres_solver_plugin solvers/ceres_solver.cpp)
target_link_libraries(ceres_solver_plugin ${catkin_LIBRARIES} 
                                          ${CERES_INCLUDE_DIRS}
                                          ${CERES_LIBRARIES}
                                          ${Boost_LIBRARIES}
                                          ${TBB_LIBRARIES}
)

#### rviz Plugin
qt_wrap_cpp(MOC_FILES rviz_plugin/slam_toolbox_rviz_plugin.hpp)
set(SOURCE_FILES rviz_plugin/slam_toolbox_rviz_plugin.cpp )
set(HEADER_FILES rviz_plugin/slam_toolbox_rviz_plugin.h )
add_library(SlamToolboxPlugin ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(SlamToolboxPlugin ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#### Tool lib for mapping
add_library(toolbox_common src/slam_toolbox_common.cpp src/map_saver.cpp src/loop_closure_assistant.cpp src/laser_utils.cpp src/slam_mapper.cpp)
target_link_libraries(toolbox_common kartoSlamToolbox ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#### Mapping executibles
add_library(async_slam_toolbox src/slam_toolbox_async.cpp)
target_link_libraries(async_slam_toolbox toolbox_common kartoSlamToolbox ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(async_slam_toolbox_node src/slam_toolbox_async_node.cpp )
target_link_libraries(async_slam_toolbox_node async_slam_toolbox)

add_library(sync_slam_toolbox src/slam_toolbox_sync.cpp)
target_link_libraries(sync_slam_toolbox toolbox_common kartoSlamToolbox ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(sync_slam_toolbox_node src/slam_toolbox_sync_node.cpp )
target_link_libraries(sync_slam_toolbox_node sync_slam_toolbox)

add_library(localization_slam_toolbox src/slam_toolbox_localization.cpp)
target_link_libraries(localization_slam_toolbox toolbox_common kartoSlamToolbox ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(localization_slam_toolbox_node src/slam_toolbox_localization_node.cpp )
target_link_libraries(localization_slam_toolbox_node localization_slam_toolbox)

add_library(lifelong_slam_toolbox src/experimental/slam_toolbox_lifelong.cpp)
target_link_libraries(lifelong_slam_toolbox toolbox_common kartoSlamToolbox ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_executable(lifelong_slam_toolbox_node src/experimental/slam_toolbox_lifelong_node.cpp )
target_link_libraries(lifelong_slam_toolbox_node lifelong_slam_toolbox)

#### Merging maps tool
add_executable(merge_maps_kinematic src/merge_maps_kinematic.cpp)
target_link_libraries(merge_maps_kinematic toolbox_common)

#### testing
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  include_directories(test)
  catkin_add_gtest(lifelong_metrics_test test/lifelong_metrics_test.cpp)
  target_link_libraries(lifelong_metrics_test lifelong_slam_toolbox)
endif()

#### Install
install(TARGETS toolbox_common
                async_slam_toolbox
                async_slam_toolbox_node
                sync_slam_toolbox
                sync_slam_toolbox_node
                localization_slam_toolbox
                localization_slam_toolbox_node
                lifelong_slam_toolbox
                lifelong_slam_toolbox_node
                ceres_solver_plugin
                merge_maps_kinematic
                SlamToolboxPlugin
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES solver_plugins.xml rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
