# ros2_robot_control

This is our lab code repository for robot control using ros2. Of course you have to install ROS2 and ros2_control first.

# package dependencies

Please clone the following packages into your ros2's workspace.

  1. franka_description (https://github.com/frankaemika/franka_description.git)

# build from source

1. install dependencies

   `sudo apt install qt6-base-dev`

2. go to your workspace's src folder and clone & build the source

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/JunchenWang/ros2_robot_control.git
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
   ```
3. launch the monitor test

   ```bash
   source ~/ros2_ws/install/local_setup.bash
   ros2 launch robot_monitor robot_monitor_launch.py
   ```
# package list

This repository contains the following packages:

### 1. robot_monitor

It is a gui interface which can display the real-time curves of joint states (position, velocity, effort, acceleration) published on the topic of "joint_states".

![1730629536341](images/README/1730629536341.png)

You can run **robot_monitor** as a standalone node to visualize the robot's states.

`ros2 run robot_monitor robot_monitor`

Again, do not forget to source the path

`source ~/ros2_ws/install/local_setup.bash`

### 2. continue...


