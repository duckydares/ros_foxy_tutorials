# ROS2 Foxy Fitzgerald Docker Images

#### Last Updated: 2021.10.25

This folder contains 2 docker environments for developing and executing robotics applications that use ROS2 Foxy Fitzgerald. The development environment has X11 port forwarding to allow for launching GUIs from the container. The executive container contains the bare setup needed to run the application.

## Dependencies
Your system will need to have the following applications installed for this container to function:
- docker
- nvidia-docker2

## Build

Before running the build script make sure that the Dockerfile
### Development Environment
`. ./build.sh ros_foxy_development`

### Executive Environment
`. ./build.sh ros_foxy_executive`

## Run
`. ./ros2_foxy_development/run.sh ros_foxy_development ~/catkin_ws`

## Join
`. ./join.sh ros2_foxy_development`