cmake_minimum_required(VERSION 2.8.3)
project(karto_sdk)

set(CMAKE_BUILD_TYPE Release) #None, Debug, Release, RelWithDebInfo, MinSizeRel
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-backtrace-limit=0")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system serialization filesystem thread)
find_package(TBB REQUIRED)
add_compile_options(-std=c++17)

catkin_package(
  DEPENDS 
    Boost
    TBB
  INCLUDE_DIRS
    include
    ${TBB_INCLUDE_DIRS}
  LIBRARIES
    kartoSlamToolbox
)

add_definitions(${EIGEN3_DEFINITIONS})

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${TBB_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_library(kartoSlamToolbox SHARED src/Karto.cpp src/Mapper.cpp src/contrib/ChowLiuTreeApprox.cpp)
target_link_libraries(kartoSlamToolbox ${Boost_LIBRARIES} ${TBB_LIBRARIES})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS kartoSlamToolbox
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(eigenExtensionsTest test/eigen_extensions_test.cpp)
  target_link_libraries(eigenExtensionsTest kartoSlamToolbox ${catkin_LIBRARIES})
  catkin_add_gtest(chowLiuTreeApproxTest test/chow_liu_tree_approx_test.cpp)
  target_link_libraries(chowLiuTreeApproxTest kartoSlamToolbox ${catkin_LIBRARIES})
endif()
