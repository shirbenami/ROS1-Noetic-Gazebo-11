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
