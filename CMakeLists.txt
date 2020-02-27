cmake_minimum_required(VERSION 3.5)
project(slam_toolbox)

add_compile_options(-std=c++14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/CMake/")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/karto_sdk/cmake)
set(CMAKE_CXX_FLAGS "-fpermissive -std=c++14")
set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(message_filters REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav2_map_server REQUIRED)
#find_package(rviz_default_plugins REQUIRED)
#find_package(rviz_rendering REQUIRED)
#find_package(rviz_common REQUIRED)
#find_package(interactive_markers REQUIRED)

#karto_sdk lib
set(BUILD_SHARED_LIBS ON)
add_subdirectory(lib/karto_sdk)

set(dependencies
  rclcpp
  message_filters
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  visualization_msgs
  pluginlib
  nav2_map_server
  tf2_geometry_msgs
  tf2_sensor_msgs
  std_msgs
  std_srvs
  builtin_interfaces
  #rviz_common
  #rviz_rendering
  #rviz_default_plugins
  #interactive_markers
)

set(libraries 
    lifelong_slam_toolbox
    localization_slam_toolbox
    sync_slam_toolbox
    async_slam_toolbox
    toolbox_common
    #SlamToolboxPlugin
    ceres_solver_plugin
)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CSparse REQUIRED)
find_package(G2O REQUIRED)
find_package(Cholmod REQUIRED)
find_package(LAPACK REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)
find_package(Boost REQUIRED system serialization filesystem thread)

include_directories(include ${EIGEN3_INCLUDE_DIRS} 
                            ${CHOLMOD_INCLUDE_DIR}
                            ${Boost_INCLUDE_DIR}
                            ${BOOST_INCLUDE_DIRS}
                            ${TBB_INCLUDE_DIRS}
                            ${CERES_INCLUDES}
)

add_definitions(${EIGEN3_DEFINITIONS})

rosidl_generate_interfaces(${PROJECT_NAME}
  srvs/Pause.srv
  srvs/ClearQueue.srv
  srvs/ToggleInteractive.srv
  srvs/Clear.srv
  srvs/SaveMap.srv
  srvs/LoopClosure.srv
  srvs/MergeMaps.srv
  srvs/AddSubmap.srv
  srvs/DeserializePoseGraph.srv
  srvs/SerializePoseGraph.srv
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs nav_msgs
)

#set(CMAKE_AUTOMOC ON)
#find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
#set(QT_LIBRARIES Qt5::Widgets Qt5::Core)
#macro(qt_wrap_ui)
#  qt5_wrap_ui(${ARGN})
#endmacro()
#macro(qt_wrap_cpp)
#  qt5_wrap_cpp(${ARGN})
#endmacro()

#### rviz Plugin
#qt_wrap_cpp(MOC_FILES rviz_plugin/slam_toolbox_rviz_plugin.h)
#set(SOURCE_FILES rviz_plugin/slam_toolbox_rviz_plugin.cpp )
#set(HEADER_FILES rviz_plugin/slam_toolbox_rviz_plugin.h )
#add_library(SlamToolboxPlugin ${SOURCE_FILES} ${HEADER_FILES})
#ament_target_dependencies(SlamToolboxPlugin
#  ${dependencies}
#)
#target_link_libraries(SlamToolboxPlugin kartoSlamToolbox ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${QT_LIBRARIES})
#rosidl_target_interfaces(SlamToolboxPlugin ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_compile_definitions(SlamToolboxPlugin PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
#pluginlib_export_plugin_description_file(slam_toolbox rviz_plugins.xml)

#### Ceres Plugin
add_library(ceres_solver_plugin solvers/ceres_solver.cpp)
ament_target_dependencies(ceres_solver_plugin ${dependencies})
target_link_libraries(ceres_solver_plugin ${CERES_LIBRARIES}
                                          ${Boost_LIBRARIES}
                                          ${TBB_LIBRARIES}
                                          kartoSlamToolbox
)
rosidl_target_interfaces(ceres_solver_plugin ${PROJECT_NAME} "rosidl_typesupport_cpp")
pluginlib_export_plugin_description_file(slam_toolbox solver_plugins.xml)

#### Tool lib for mapping
add_library(toolbox_common src/slam_toolbox_common.cpp src/map_saver.cpp src/loop_closure_assistant.cpp src/laser_utils.cpp src/slam_mapper.cpp)
ament_target_dependencies(toolbox_common
  ${dependencies}
)
target_link_libraries(toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
rosidl_target_interfaces(toolbox_common ${PROJECT_NAME} "rosidl_typesupport_cpp")

#### Mapping executibles
add_library(async_slam_toolbox src/slam_toolbox_async.cpp)
target_link_libraries(async_slam_toolbox toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
add_executable(async_slam_toolbox_node src/slam_toolbox_async_node.cpp )
target_link_libraries(async_slam_toolbox_node async_slam_toolbox)

add_library(sync_slam_toolbox src/slam_toolbox_sync.cpp)
target_link_libraries(sync_slam_toolbox toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
add_executable(sync_slam_toolbox_node src/slam_toolbox_sync_node.cpp )
target_link_libraries(sync_slam_toolbox_node sync_slam_toolbox)

add_library(localization_slam_toolbox src/slam_toolbox_localization.cpp)
target_link_libraries(localization_slam_toolbox toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
add_executable(localization_slam_toolbox_node src/slam_toolbox_localization_node.cpp )
target_link_libraries(localization_slam_toolbox_node localization_slam_toolbox)

add_library(lifelong_slam_toolbox src/experimental/slam_toolbox_lifelong.cpp)
target_link_libraries(lifelong_slam_toolbox toolbox_common kartoSlamToolbox ${Boost_LIBRARIES})
add_executable(lifelong_slam_toolbox_node src/experimental/slam_toolbox_lifelong_node.cpp )
target_link_libraries(lifelong_slam_toolbox_node lifelong_slam_toolbox)

#### Merging maps tool
add_executable(merge_maps_kinematic src/merge_maps_kinematic.cpp)
target_link_libraries(merge_maps_kinematic kartoSlamToolbox toolbox_common)

#### testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  include_directories(test)
  ament_add_gtest(lifelong_metrics_test test/lifelong_metrics_test.cpp)
  target_link_libraries(lifelong_metrics_test lifelong_slam_toolbox)
endif()

#### Install
install(TARGETS async_slam_toolbox_node
                sync_slam_toolbox_node
                localization_slam_toolbox_node
                lifelong_slam_toolbox_node
                merge_maps_kinematic
                ${libraries}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

install(FILES solver_plugins.xml #rviz_plugins.xml
  DESTINATION share
)

ament_export_include_directories(include)
ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies})
ament_package()
