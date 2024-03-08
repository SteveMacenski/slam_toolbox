cmake_minimum_required(VERSION 3.5)
project(slam_toolbox_msgs)

set(MSG_DEPS std_srvs std_msgs geometry_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation ${MSG_DEPS})

add_service_files(DIRECTORY srv
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
    Reset.srv
)

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS message_runtime ${MSG_DEPS})
