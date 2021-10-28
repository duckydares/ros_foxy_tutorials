# ROS2 Foxy Fitzgerald Development Dockerfile
# Author: Ducky
# Updated: 2021.10.21

# Base
FROM ubuntu:20.04

# Timezone data needs to be installed first and separately
# Set to EST : New_York
RUN export DEBIAN_FRONTEND=noninteractive \
 && apt-get update -qq \
 && apt-get install -y -qq \
    tzdata \
 && ln -fs /usr/share/zoneinfo/America/New_York /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt-get clean -qq
 
# Development Tools
RUN apt-get update -qq \
 && apt-get install -y -qq \
        build-essential \
        bwm-ng \
        cmake \
        cppcheck \
        gdb \
        git \
        g++-8 \
        libbluetooth-dev \
        libccd-dev \
        libcwiid-dev \
        libfcl-dev \
        libgoogle-glog-dev \
        libspnav-dev \
        libusb-dev \
        lsb-release \
        python3-dbg \
        python3-empy \
        python3-numpy \
        python3-setuptools \
        python3-pip \
        python3-venv \
        software-properties-common \
        sudo \
        vim \
        wget \
        net-tools \
        iputils-ping \
        libyaml-cpp-dev \
 && apt-get clean -qq

# Text Editor : Default = Sublime
RUN wget -qO - https://download.sublimetext.com/sublimehq-pub.gpg | apt-key add - && \
  apt-get install apt-transport-https && \
  echo "deb https://download.sublimetext.com/ apt/stable/" | tee /etc/apt/sources.list.d/sublime-text.list && \
  apt-get update && \
  apt-get install -yy sublime-text

# Install Nvidia GL library based on Nvidia driver
# TODO: Make nvidia_version be the version of the driver on system
# CURRENT: Must manually change version based on 'nvidia-smi'
ARG nvidia_version
RUN apt-get update && \
  apt-get install -y libnvidia-gl-470 && \
  apt-get clean -qq

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`
ARG user_id
ENV USERNAME developer
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

# Commands below run as the developer user
USER $USERNAME

# Make a couple folders for organizing docker volumes
RUN mkdir ~/workspaces ~/other

# When running a container start in the developer's home folder
WORKDIR /home/$USERNAME

# ROS2 Install
RUN sudo apt-get update && sudo apt-get install -y curl gnupg2 lsb-release
RUN sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN sudo echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN sudo apt-get update

RUN sudo DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration

RUN sudo apt-get install -y ros-foxy-desktop

RUN sudo apt-get install -y ros-foxy-turtlesim \
  ~nros-foxy-rqt*

