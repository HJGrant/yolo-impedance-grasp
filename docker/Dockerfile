# Start from the image you pulled
FROM tum-ics-ros:noetic-ics-msbrdm-devel-vscode

ARG ROS_DISTRO=noetic

# Switch to root to install things
USER root

# 1. Install vision_msgs from the internet
RUN apt-get update && apt-get install -y \
    ros-$ROS_DISTRO-vision-msgs \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-rgbd-launch \
    ros-$ROS_DISTRO-aruco-ros \
    && rm -rf /var/lib/apt/lists/*

RUN pip install -U ultralytics --no-cache-dir

RUN printf "\n\
    rosmaster_ur10(){\n\
        export ROS_MASTER_URI=http://192.168.1.3:11311\n\
        export ROS_IP=192.168.1.3\n\
    }\n\
    \n\
    rosmaster_local(){\n\
        export ROS_MASTER_URI=http://localhost:11311\n\
        export ROS_IP=127.0.0.1\n\
    }\n" >> /home/devel/.bashrc
    
RUN pip install open3d \
                pyransac3d \
                --no-cache-dir

# Switch back to the 'devel' user so you aren't root inside the terminal
USER devel