# Start from the image you pulled
FROM tum-ics-ros:noetic-ics-msbrdm-devel-vscode

ARG ROS_DISTRO=noetic

# Switch to root to install things
USER root

# 1. Install vision_msgs from the internet
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

RUN pip install -U ultralytics --no-cache-dir


# Switch back to the 'devel' user so you aren't root inside the terminal
USER devel