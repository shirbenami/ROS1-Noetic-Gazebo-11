# 🚁 Hector Quadrotor - Keyboard Control Setup Guide

This guide explains how to control a simulated quadrotor using your keyboard inside the Hector Quadrotor Gazebo simulation (ROS Melodic).
It walks through installing dependencies, setting up the keyboard bridge, enabling motors, and successfully flying the drone using keys.

---

## 🧩 Prerequisites

* ROS Melodic installed (tested on Ubuntu 18.04)
* `hector_quadrotor` stack cloned into `~/catkin_ws/src`
* Workspace built and sourced:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 🏗️ Step-by-Step Instructions

### 1. Launch the Hector Quadrotor Indoor Simulation

```bash
roslaunch hector_quadrotor_demo indoor_slam_gazebo.launch
```

> 📝 This will load the drone in a Gazebo indoor environment.

---

### 2. Create a Keyboard Teleoperation Launch File

Create a file at:

```bash
cd ~/catkin_ws/src/hector_quadrotor/hector_quadrotor_teleop/launch
nano keyboard_teleop_quadrotor.launch
```

Paste the following content:

```xml
<launch>
  <arg name="control_mode" default="velocity" />

  <node name="keyboard_joy" pkg="teleop_tools" type="key_teleop" output="screen">
    <param name="scale_linear" value="1.0" />
    <param name="scale_angular" value="1.0" />
  </node>

  <node name="teleop" pkg="hector_quadrotor_teleop" type="quadrotor_teleop" output="screen">
    <rosparam subst_value="true">
      control_mode: $(arg control_mode)
      x_axis: 4
      y_axis: 3
      z_axis: 2
      thrust_axis: 2
      yaw_axis: 1
      slow_button: 6
      go_button: 4
      stop_button: 2
    </rosparam>
  </node>
</launch>
```

---

### 3. Install `key_teleop` and Dependencies

```bash
cd ~/catkin_ws/src
git clone https://github.com/ros-teleop/teleop_tools.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

### 4. Launch the Keyboard Teleop

Open a **new terminal**, then:

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch hector_quadrotor_teleop keyboard_teleop_quadrotor.launch
```

If no error appears and the `teleop` node is active, you're ready to fly! 🚀

---

### 5. Enable Motors - Takeoff Action

The motors are disabled by default. To activate them, run:

```bash
rostopic pub /command/takeoff std_msgs/Empty "{}"
```

> 🛑 You **must** take off before trying to control the drone with keyboard!

---

### 6. Fly Using Keyboard

Use the keyboard to move the drone:

* Arrow keys or ASWD for translation
* Space, Shift, or other keys as configured to adjust thrust or yaw

📌 Check `key_teleop` documentation to configure specific key bindings if needed.

---

## ✅ Tips

* Use `rqt_graph` to debug connections between nodes
* Always check `/command/takeoff` was triggered if drone isn't moving
* Run `rostopic echo /command/twist` to verify messages are being published

---

## 📦 Related Packages

* `hector_quadrotor_teleop`
* `teleop_tools` (contains `key_teleop`)
* `geometry_msgs/TwistStamped`

---

## 🧪 Optional Debug Commands

Check active nodes:

```bash
rosnode list
```

Check topic info:

```bash
rostopic info /command/twist
```

Kill stuck nodes:

```bash
rosnode kill /pose_action
```

---

## 🥳 That's It!

You now have full control of the simulated Hector Quadrotor using your keyboard.
