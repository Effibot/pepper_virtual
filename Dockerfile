# Base image
FROM osrf/ros:kinetic-desktop-full

MAINTAINER Finn Rietz <5rietz@informatik.uni-hamburg.de>

# (RUN is executed when image is build)
RUN apt-get update
RUN apt-get install -y vim wget libprotobuf-dev protobuf-compiler

# packages required for Pepper Gazebo simulation
RUN apt-get install -y ros-kinetic-tf2-sensor-msgs ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-plugins ros-kinetic-controller-manager ros-kinetic-ddynamic-reconfigure-python 

# install pepper meshes. We have to pipe this into 'yes' to agree to the license. otherwise docker build get's stuck on this step...
# we also have these debian environment params, otherwise the yes still gets stuck on the prompt in the mesh installation
ENV DEBIAN_FRONTEND noninteractive
ENV DEBIAN_FRONTEND teletype
RUN yes | apt-get install ros-kinetic-pepper-meshes

# add the catkin ws from this folder in the container
ADD ./catkin_ws /catkin_ws

# build the catkin_ws inside the container
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash && cd /catkin_ws && catkin_make'

# # finally, source the workspace
# # for this, we override the default shell command (bash) that is invoked in the container!
SHELL ["/bin/bash", "-c", "source /catkin_ws/devel/setup.bash && source /ros_entrypoint.sh"]
