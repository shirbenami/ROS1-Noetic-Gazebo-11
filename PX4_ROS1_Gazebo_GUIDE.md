
```
nano Dockerfile
```

```
# PX4 + ROS1 + Gazebo Dockerfile (Melodic, PX4 SITL, Iris Drone)

FROM ros:melodic

ENV DEBIAN_FRONTEND=noninteractive

# System Dependencies
RUN apt update && apt install -y \
    git wget nano curl lsb-release gnupg2 \
    python3-pip python3-dev \
    build-essential \
    cmake ninja-build \
    libxml2-dev libxslt-dev \
    libgazebo9-dev libeigen3-dev libopencv-dev \
    protobuf-compiler \
    libprotobuf-dev libprotoc-dev \
    libcurl4-openssl-dev libzmq3-dev \
    libboost-all-dev \
    python-rosdep python-catkin-tools \
    ros-melodic-desktop-full \
    ros-melodic-gazebo-ros-pkgs \
    ros-melodic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# Python Dependencies for PX4
RUN pip3 install --upgrade pip && \
    pip3 install \
    cython \
    empy==3.3.4 \
    kconfiglib \
    toml jinja2 \
    numpy \
    pyserial \
    pyyaml==5.4.1 \
    future \
    pandas \
    lxml

# ROS setup
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN bash -c 'source /opt/ros/melodic/setup.bash && catkin_make'
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

# PX4 Source
WORKDIR /root/catkin_ws
RUN git clone https://github.com/PX4/PX4-Autopilot.git --recursive
WORKDIR /root/catkin_ws/PX4-Autopilot
RUN bash ./Tools/setup/ubuntu.sh || true

# Optional: Prebuild PX4
# RUN make px4_sitl_default gazebo

ENV DISPLAY=:0

CMD ["/bin/bash"]
```

```
docker build -t px4-ros1-gazebo .
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
  px4-ros1-gazebo
```
```
cd ~/catkin_ws/PX4-Autopilot
make px4_sitl_default gazebo
```

