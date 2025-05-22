# 🐢 ROS1 Noetic + Gazebo 11 + TurtleBot3 Simulation

This repository provides a full Docker-based simulation environment with:

- ✅ ROS1 Noetic
- ✅ Gazebo 11 with GUI support
- ✅ `gazebo_ros_pkgs` integration
- ✅ TurtleBot3 simulation + teleoperation

---

## 🚀 Quick Start

### 🐳 1. Clone this repository

```bash
git clone [https://github.com/<your-username>/<your-repo-name>](https://github.com/shirbenami/ROS1-Noetic-Gazebo-11).git
cd <your-repo-name>
```

### 🛠️ 2. Build the Docker Image
```bash
docker build -t ros1-gazebo .
```

### 🖥️ 3. Enable X11 GUI for Gazebo (on Ubuntu only)
```bash
xhost +local:docker
```

### ▶️ 4. Run the Docker Container
```bash
docker run -it \
  --name ros1-gazebo-container \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --privileged \
  ros1-gazebo
```

### 🤖 5. Inside the Container: Setup TurtleBot3
```bash
chmod +x setup_turtlebot3.sh
./setup_turtlebot3.sh
```

This will:

* Clone TurtleBot3 + Simulations

* Install dependencies

* Compile workspace

* Launch turtlebot3_world.launch in Gazebo

### 🎮 6. Control the Robot (Teleoperation)
In another terminal:

```bash
docker exec -it ros1-gazebo-container bash
source ~/.bashrc
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Use keys i, j, l, , to move the robot.




______________________________________________________________________________________________________________
______________________________________________________________________________________________________________
______________________________________________________________________________________________________________
______________________________________________________________________________________________________________
______________________________________________________________________________________________________________

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
```

## 🧱 Step 2: Build the Docker Image

From the project directory, build the Docker image:

```bash
docker build -t ros1-gazebo .
```

## 🖥️ Step 3: Run the Container with GUI Support
If you're running on a local Ubuntu machine:

```bash
xhost +local:docker
```

Then run the container:
```bash
docker run -it \
  --name ros1-gazebo-container \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --privileged \
  ros1-gazebo
```

## 🧪 Step 4: Test the Setup
Inside the container:
```bash
roslaunch gazebo_ros empty_world.launch
```

## 🤖 Step 5: Add TurtleBot3 Simulation
To automatically install and run the TurtleBot3 simulation, use the following script.

🧾 File: setup_turtlebot3.sh
Inside the container, create the script:

```bash
nano setup_turtlebot3.sh
Paste the following content:
```
```bash
#!/bin/bash

echo "📦 Creating catkin_ws"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

echo "🌐 Cloning TurtleBot3 packages from GitHub"
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

echo "🔧 Running catkin_make"
cd ~/catkin_ws
catkin_make

echo "📥 Installing dependencies with rosdep"
rosdep update
rosdep install --from-paths src --ignore-src -r -y

echo "🧠 Setting TurtleBot3 model to 'burger'"
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
export TURTLEBOT3_MODEL=burger

echo "📦 Final compilation"
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash

echo "🚀 Launching TurtleBot3 simulation"
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Make it executable and run:
```bash
chmod +x setup_turtlebot3.sh
./setup_turtlebot3.sh
```
