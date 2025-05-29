
```
nano Dockerfile
```

```
FROM ubuntu:18.04

# Avoid interactive dialogs
ENV DEBIAN_FRONTEND=noninteractive

# Locale & tz
RUN apt-get update && apt-get install -y \
    locales tzdata && \
    locale-gen en_US.UTF-8

ENV LANG=en_US.UTF-8 \
    LANGUAGE=en_US:en \
    LC_ALL=en_US.UTF-8

# Base dependencies
RUN apt-get update && apt-get install -y \
    sudo curl wget git gnupg2 lsb-release \
    build-essential cmake ninja-build \
    python3-pip python3-yaml python3-jinja2 \
    python3-empy python3-dev python3-setuptools \
    python-is-python3 python3-future \
    libxml2-utils

# ROS1 Melodic installation
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y \
    ros-melodic-desktop-full ros-melodic-mavros ros-melodic-mavros-extras \
    ros-melodic-tf2-sensor-msgs ros-melodic-geographic-msgs \
    ros-melodic-octomap* \
    geographiclib-tools && \
    rosdep init && rosdep update && \
    geographiclib-get-geoids egm96-5

# Gazebo Classic 9
RUN apt-get install -y \
    gazebo9 libgazebo9-dev \
    libgazebo9-plugins-dev \
    libgazebo9-msgs-dev \
    libgazebo9-transport-dev \
    libeigen3-dev libopencv-dev

# PX4 Autopilot
WORKDIR /root/catkin_ws
RUN git clone https://github.com/PX4/PX4-Autopilot.git && \
    cd PX4-Autopilot && \
    git submodule update --init --recursive

# Environment setup
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_ws/PX4-Autopilot/Tools/setup_gazebo.bash /root/catkin_ws/PX4-Autopilot /root/catkin_ws/PX4-Autopilot/build/px4_sitl_default" >> ~/.bashrc && \
    echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/catkin_ws/PX4-Autopilot" >> ~/.bashrc && \
    echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/root/catkin_ws/PX4-Autopilot/Tools/sitl_gazebo" >> ~/.bashrc

# Build PX4 + SITL for Gazebo
WORKDIR /root/catkin_ws/PX4-Autopilot
RUN pip3 install --upgrade pip && \
    pip3 install pyyaml
RUN make px4_sitl_default gazebo

CMD ["bash"]

```

```
docker build -t ros1-px4-gazebo .
```

```
xhost +local:docker
```

```
docker run -it \
  --name px4-container \
  --net=host \
  --privileged \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  ros1-px4-gazebo
```


