cmake_minimum_required(VERSION 2.8.3)
project(slam_karto)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake/")

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    message_filters
    nav_msgs
    open_karto
    rosconsole
    roscpp
    sensor_msgs
    sparse_bundle_adjustment
    tf
    visualization_msgs
    pluginlib
)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CSparse REQUIRED)
find_package(G2O REQUIRED)
find_package(Cholmod REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} 
                            ${EIGEN3_INCLUDE_DIRS} 
                            ${CHOLMOD_INCLUDE_DIR}
)

add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES spa_solver_plugin g2O_solver_plugin
    CATKIN_DEPENDS
      message_filters
      nav_msgs
      open_karto
      rosconsole
      roscpp
      sparse_bundle_adjustment
      sensor_msgs
      tf
      visualization_msgs
      pluginlib
)

add_library(spa_solver_plugin solvers/spa_solver.cpp)
target_link_libraries(spa_solver_plugin ${catkin_LIBRARIES})

add_compile_options(-std=c++11)
add_library(g2O_solver_plugin solvers/g2o_solver.cpp)
target_link_libraries(g2O_solver_plugin 
  ${catkin_LIBRARIES}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_TYPES_SLAM2D}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CSPARSE_EXTENSION}
  ${CHOLMOD_LIBRARIES}
  ${CSPARSE_LIBRARY}
)

# I hate this hack, but SPA libraries need 98 but g2o needs 11
add_compile_options(-std=c++98) 
add_executable(slam_karto src/slam_karto.cpp)
target_link_libraries(slam_karto ${catkin_LIBRARIES}  )

install(TARGETS slam_karto spa_solver_plugin g2O_solver_plugin
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

install(FILES solver_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)