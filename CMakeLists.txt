cmake_minimum_required(VERSION 2.8.3)
project(slam_karto)

add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake/")
set(CMAKE_CXX_FLAGS "-fpermissive -std=c++11")                                                                            
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive") 

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
    message_generation
    std_srvs
    std_msgs
    interactive_markers
)

find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CSparse REQUIRED)
find_package(G2O REQUIRED)
find_package(Cholmod REQUIRED)
find_package(Ceres REQUIRED COMPONENTS SuiteSparse)

include_directories(include ${catkin_INCLUDE_DIRS} 
                            ${EIGEN3_INCLUDE_DIRS} 
                            ${CHOLMOD_INCLUDE_DIR}
                            #${MKL_INCLUDE_DIR}
                            #${GTSAM_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

add_service_files(DIRECTORY srvs
  FILES
  Pause.srv
  ClearQueue.srv
  ToggleInteractive.srv
  Clear.srv
  SaveMap.srv
)
generate_messages(
  DEPENDENCIES
  std_srvs
  std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES spa_solver_plugin g2O_solver_plugin ceres_solver_plugin
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
      message_runtime
      std_srvs
      std_msgs
      interactive_markers
)

#### SPA
add_library(spa_solver_plugin solvers/spa_solver.cpp)
target_link_libraries(spa_solver_plugin ${catkin_LIBRARIES})

#### Ceres
add_library(ceres_solver_plugin solvers/ceres_solver.cpp)
target_link_libraries(ceres_solver_plugin ${catkin_LIBRARIES} 
                                          ${CERES_INCLUDE_DIRS}
                                          ${CERES_LIBRARIES}
)

#### G2O
add_library(g2O_solver_plugin solvers/g2o_solver.cpp)
target_link_libraries(g2O_solver_plugin ${catkin_LIBRARIES}
                                        ${G2O_CORE_LIBRARY}
                                        ${G2O_STUFF_LIBRARY}
                                        ${G2O_TYPES_SLAM2D}
                                        ${G2O_SOLVER_CHOLMOD}
                                        ${G2O_SOLVER_CSPARSE}
                                        ${G2O_SOLVER_CSPARSE_EXTENSION}
                                        ${CHOLMOD_LIBRARIES}
                                        ${CSPARSE_LIBRARY}
)

add_executable(slam_karto src/slam_karto.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(slam_karto ${catkin_LIBRARIES})

install(TARGETS slam_karto spa_solver_plugin g2O_solver_plugin ceres_solver_plugin
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
