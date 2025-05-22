FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    sudo curl git wget nano lsb-release gnupg2 \
    python3-pip \
    python3-rosdep python3-catkin-tools \
    ros-noetic-desktop-full \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/catkin_ws/src && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make"

RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

ENV DISPLAY=:0
