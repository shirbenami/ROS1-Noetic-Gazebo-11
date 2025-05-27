mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

git clone -b noetic-devel https://github.com/R$
git clone -b noetic-devel https://github.com/R$

cd ~/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src -$

catkin_make

echo "export TURTLEBOT3_MODEL=waffle" >> ~/.ba$
export TURTLEBOT3_MODEL=waffle

source /opt/ros/melodic/setup.bash
source devel/setup.bash

roslaunch turtlebot3_gazebo turtlebot3_world.l$



