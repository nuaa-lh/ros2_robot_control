# ros2_robot_control

This is our lab code repository for robot control using ros2. Of course you have to install ROS2  first. The codes are developed under **Ubuntu 24** with **ROS2 Jazzy**.

# package dependencies

Please clone the following packages into the src folder of your ros2's workspace. (e.g., ~/ros2_ws)

1. franka_description (https://github.com/frankaemika/franka_description.git)

# build from source

1. install dependencies

   ```bash
   sudo apt install qt6-base-dev qt6-charts-dev libjsoncpp-dev
   ```
2. go to your workspace's src folder and clone & build the source

   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/JunchenWang/ros2_robot_control.git
   rosdep install --from-paths src -y --ignore-src
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install
   ```
3. launch the monitor test

   ```bash
   source ~/ros2_ws/install/local_setup.bash
   ros2 launch robot_monitor robot_monitor_launch.py
   ```

Now, you can drag the sliders on the joint_state_publisher_gui to watch the position curves of each joint.

# package list

This repository contains the following packages:

1. robot_monitor

   It is a gui interface which can display the real-time curves of joint states (position, velocity, effort, acceleration) published on the topic of "joint_states".

   ![1730629536341](images/README/1730629536341.png)

   You can run **robot_monitor** as a standalone node to visualize the robot's states.

   ```bash
   ros2 run robot_monitor robot_monitor
   ```
   before that, do not forget to source the path

   ```bash
   source ~/ros2_ws/install/local_setup.bash
   ```
   Now, if some other node on the ROS2 network published messages into the "joint_states" topic, the **robot_monitor** can capture the signals.
2. robot_math

   It is a libray for robot mathematics, including dynamics and kinematics using twist and wrench representation. The codes in the matlab_code folder are generated using Matlab coder generation.
