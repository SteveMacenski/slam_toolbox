FROM ros:eloquent-ros-base-bionic

# USE BASH
SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends apt-utils

# slam_toolbox
RUN mkdir -p colcon_ws/src
RUN cd colcon_ws/src && git clone -b eloquent-devel https://github.com/SteveMacenski/slam_toolbox.git
RUN source /opt/ros/eloquent/setup.bash \
    && cd colcon_ws \
    && rosdep update \
    && rosdep install -y -r --from-paths src --ignore-src --rosdistro=eloquent -y

RUN source /opt/ros/eloquent/setup.bash \
    && cd colcon_ws/ \
    && colcon build  --cmake-args=-DCMAKE_BUILD_TYPE=Release \
    && colcon test
