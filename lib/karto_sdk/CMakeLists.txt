cmake_minimum_required(VERSION 3.5)
project(karto_sdk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-backtrace-limit=0")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system serialization filesystem thread)
find_package(TBB REQUIRED)

set(dependencies
  rclcpp
)

include_directories(include ${catkin_INCLUDE_DIRS} 
                            ${EIGEN3_INCLUDE_DIRS} 
                            ${Boost_INCLUDE_DIR}
                            ${BOOST_INCLUDE_DIRS}
                            ${TBB_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${TBB_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_library(kartoSlamToolbox SHARED src/Karto.cpp src/Mapper.cpp)
ament_target_dependencies(kartoSlamToolbox ${dependencies})
target_link_libraries(kartoSlamToolbox ${Boost_LIBRARIES} ${TBB_LIBRARIES})

install(DIRECTORY include/
	DESTINATION include/
)

install(TARGETS kartoSlamToolbox
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(kartoSlamToolbox)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
