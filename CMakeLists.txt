cmake_minimum_required(VERSION 2.8.3)
project(slam_toolbox)

add_compile_options(-std=c++17)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake/")
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
    message_generation
    std_srvs
    std_msgs
    tf2_geometry_msgs
    interactive_markers
    rviz
)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(CSparse REQUIRED)
find_package(G2O REQUIRED)
find_package(Cholmod REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)

catkin_package(
    INCLUDE_DIRS include
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
      std_srvs
      std_msgs
      tf2_geometry_msgs
      interactive_markers
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)
find_package(Boost REQUIRED system serialization filesystem thread)

include_directories(include ${catkin_INCLUDE_DIRS} 
                            ${EIGEN3_INCLUDE_DIRS} 
                            ${CHOLMOD_INCLUDE_DIR}
                            ${Boost_INCLUDE_DIR}
)

add_definitions(${EIGEN3_DEFINITIONS})

add_service_files(DIRECTORY srvs
  FILES
  Pause.srv
  ClearQueue.srv
  ToggleInteractive.srv
  Clear.srv
  SaveMap.srv
  LoopClosure.srv
  MergeMaps.srv
  AddSubmap.srv
  DeserializePoseGraph.srv
  SerializePoseGraph.srv
)

generate_messages(
  DEPENDENCIES
  std_srvs
  std_msgs
  geometry_msgs
)

#### Ceres Plugin
add_library(ceres_solver_plugin solvers/ceres_solver.cpp)
target_link_libraries(ceres_solver_plugin ${catkin_LIBRARIES} 
                                          ${CERES_INCLUDE_DIRS}
                                          ${CERES_LIBRARIES}
)

#### rviz Plugin
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
qt_wrap_cpp(MOC_FILES rviz_plugin/slam_toolbox_rviz_plugin.hpp)
set(SOURCE_FILES rviz_plugin/slam_toolbox_rviz_plugin.cpp )
set(HEADER_FILES rviz_plugin/slam_toolbox_rviz_plugin.h )
add_library(slam_toolbox_rviz_plugin ${SOURCE_FILES} ${HEADER_FILES})
add_dependencies(slam_toolbox_rviz_plugin ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(slam_toolbox_rviz_plugin ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#### Main lib for mapping
add_library(toolbox_lib src/slam_toolbox.cpp src/map_saver.cpp src/loop_closure_assistant.cpp src/laser_utils.cpp)
add_dependencies(toolbox_lib ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(toolbox_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#### Main tools for mapping
add_executable(slam_toolbox src/slam_toolbox_node.cpp )
target_link_libraries(slam_toolbox toolbox_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#### Merging maps tool
add_executable(merge_maps_kinematic src/merge_maps_kinematic.cpp)
target_link_libraries(merge_maps_kinematic toolbox_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#### Install
install(TARGETS toolbox_lib 
                slam_toolbox
                ceres_solver_plugin
                merge_maps_kinematic
                slam_toolbox_rviz_plugin
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
