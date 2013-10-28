cmake_minimum_required(VERSION 2.8.3)
project(slam_karto)

# should also find_package open_karto but it isn't a proper catkin package yet
set(ROS_DEPS tf sensor_msgs visualization_msgs roscpp rosconsole nav_msgs message_filters sba open_karto)

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPS})
find_package(Eigen REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
)

add_executable(slam_karto src/slam_karto.cpp src/spa_solver.cpp)
target_link_libraries(slam_karto ${catkin_LIBRARIES})
