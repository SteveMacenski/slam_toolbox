cmake_minimum_required(VERSION 3.5)
project(slam_toolbox_rviz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED
  COMPONENTS
    rviz
    slam_toolbox_msgs
)
include_directories(${catkin_INCLUDE_DIRS})

add_definitions(-DQT_NO_KEYWORDS)


#include_directories(${EIGEN3_INCLUDE_DIRS})
#add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS
      ${EIGEN3_INCLUDE_DIRS}
    CATKIN_DEPENDS
      rviz
      slam_toolbox_msgs
)

#set(CMAKE_AUTOMOC ON)
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

set(CMAKE_INCLUDE_CURRENT_DIR ON)


set(SOURCE_FILES 
  src/slam_toolbox_rviz_plugin.cpp 
)

qt_wrap_cpp(${PROJECT_NAME} MOC_FILES 
  src/slam_toolbox_rviz_plugin.h
)

add_library(${PROJECT_NAME} ${SOURCE_FILES} ${MOC_FILES})
target_link_libraries(${PROJECT_NAME} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${QT_LIBRARIES} ${catkin_LIBRARIES})

#### Install
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
