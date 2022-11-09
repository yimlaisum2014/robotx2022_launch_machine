# Making CPU docker image
*You should run this code on a CPU computer*

Base image : osrf/ros:melodic-desktop-full

Dependencies: 
- ROS(Melodic)
- Python 3.6
- Pyrobot
- rosbridge-server
- Smach

Usage:

You can skip build docker images, just source docker_run.sh, then it will automatically pull docker image from Docker Hub. The building docker image is used for developer.
