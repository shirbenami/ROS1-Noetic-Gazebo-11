
```
nano Dockerfile
```

```
ROM ubuntu:18.04

# Set locale
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# Set noninteractive frontend
ENV DEBIAN_FRONTEND=noninteractive

# Base dependencies
RUN apt-get update && apt-get install -y \
    sudo curl wget git gnupg2 lsb-release \
    build-essential cmake ninja-build \
    python3-pip python3-yaml python3-jinja2 \
    python3-empy python3-dev python3-setuptools \
    python3-future libxml2-utils \
    locales tzdata net-tools iputils-ping \
    && [ ! -e /usr/bin/python ] && ln -s /usr/bin/python3 /usr/bin/python || true \
    && locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Install ROS
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list>
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - &>
    apt-get update && \
    apt-get install -y ros-melodic-desktop-full
# Source ROS setup
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Install ROS dependencies
RUN apt-get install -y \
    python3-rosdep python3-rosinstall-generator python3-vcstool python3-rosinstall \
    ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control

# Initialize rosdep
RUN rosdep init && rosdep update

# Setup working directory
WORKDIR /root/catkin_ws

# Done
CMD ["/bin/bash"]

```

```
docker build -t ros1-px4-gazebo -f Dockerfile.ros1-px4-gazebo .
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


