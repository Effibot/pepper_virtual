# Base image
FROM osrf/ros:kinetic-desktop-full

MAINTAINER Finn Rietz <5rietz@informatik.uni-hamburg.de>

# (RUN is executed when image is build)
RUN apt-get update
RUN apt-get install -y vim wget libprotobuf-dev protobuf-compiler

# packages required for Pepper Gazebo simulation
RUN apt-get install -y ros-kinetic-tf2-sensor-msgs ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-gazebo-ros ros-kinetic-gazebo-ros-control ros-kinetic-gazebo-plugins ros-kinetic-controller-manager ros-kinetic-ddynamic-reconfigure-python ros-kinetic-gmapping ros-kinetic-map-server 

# install pepper meshes. We have to pipe this into 'yes' to agree to the license. otherwise docker build get's stuck on this step...
# we also have these debian environment params, otherwise the yes still gets stuck on the prompt in the mesh installation
ENV DEBIAN_FRONTEND noninteractive
ENV DEBIAN_FRONTEND teletype
RUN yes | apt-get install ros-kinetic-pepper-meshes

# add the catkin ws from this folder in the container
# ADD syntax: <host-path> <container-path>
ADD ./catkin_ws /catkin_ws

# add the saved maps folder from the physical directory to the workspac
ADD ./saved_maps /saved_maps

# build the catkin_ws inside the container
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash && cd /catkin_ws && catkin_make'

# we add these two commands to the bashrc in the container, so that the entrypoint and workspacea will be sourced,
# whenever a new bash session is instantiated in the container
RUN echo 'source /ros_entrypoint.sh' >>  /root/.bashrc
RUN echo 'source /catkin_ws/devel/setup.bash' >> /root/.bashrc
