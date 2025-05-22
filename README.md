# 🐳 ROS1 Noetic + Gazebo 11 Docker Setup Guide

This guide walks you through creating a Docker-based development environment with:

✅ ROS1 Noetic  
✅ Gazebo 11  
✅ `gazebo_ros_pkgs` for ROS-Gazebo integration  
✅ GUI support (launch Gazebo with full display)  
✅ 🐢 TurtleBot3 full simulation and teleoperation  

---

## 📁 Step 1: Create Project Folder and Dockerfile

First, create a working directory and a `Dockerfile`:

```bash
mkdir -p ~/ros1_gazebo_docker && cd ~/ros1_gazebo_docker
touch Dockerfile

FROM ros:noetic

ENV DEBIAN_FRONTEND=noninteractive

# Install basic tools and ROS packages
RUN apt update && apt install -y \
    sudo curl git wget nano lsb-release gnupg2 \
    python3-pip \
    python3-rosdep python3-catkin-tools \
    ros-noetic-desktop-full \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# Create Catkin workspace
RUN mkdir -p /root/catkin_ws/src && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make"

# Source ROS and Catkin workspace on container start
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# Enable GUI support
ENV DISPLAY=:0

## 🧱 Step 2: Build the Docker Image

From the project directory, build the Docker image:

```bash
docker build -t ros1-gazebo .

## Step 3: Run the Container with GUI Support
If you're running on a local Ubuntu machine:

```bash
xhost +local:docker

Then run the container:
```bash
docker run -it \
  --name ros1-gazebo-container \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --privileged \
  ros1-gazebo
