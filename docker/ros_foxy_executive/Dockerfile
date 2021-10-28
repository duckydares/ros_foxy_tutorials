# ROS2 Foxy Fitzgerald Development Dockerfile
# Author: Ducky
# Updated: 2021.10.21

# Base
FROM ubuntu:20.04

# ROS2 Install
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update

RUN DEBIAN_FRONTEND="noninteractive" apt-get install -y tzdata

RUN DEBIAN_FRONTEND="noninteractive" apt-get install -y keyboard-configuration

RUN apt-get install -y ros-foxy-ros-base

# Copy over code from base repositories

# Build repositories
# colcon build --install

# Run entrypoint script
COPY ros_foxy_executive/entrypoint.bash ./
ENTRYPOINT ["./entrypoint.bash"]