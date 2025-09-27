# Go2-Software
Software for the Unitree Go2 quadruped robot

This repository contains the files required to control the **GO2** robot with ROS 2 Foxy.  
Execution is divided into two parts: **Local PC** (visualization and navigation) and **Robot** (sensors and drivers).

---

## 🚀 Execution Steps

```bash
# Configure a wired connection with IP 192.168.123.X, netmask 255.255.255.0 DGateway 192.168.123.1
# 0. Connect to the robot via SSH
ssh unitree@192.168.123.18
# password: 123
source /opt/ros/foxy/setup.bash   # load ROS Foxy on the robot

# 1. Copy the Robot folder to the robot (from the local PC, outside SSH)
scp -r /path/to/repo/GO2/Robot unitree@192.168.123.18:/home/unitree/

# 2. Build both workspaces

## On the local PC:
source /opt/ros/foxy/setup.bash
cd ~/go2_ws
colcon build --symlink-install
source install/setup.bash

## On the robot:
ssh unitree@192.168.123.18
source /opt/ros/foxy/setup.bash
cd ~/go2_robot_ws
colcon build --symlink-install
source install/setup.bash

# 3. Launch on the PC (main stack with RViz)
ros2 launch surge_et_ambula go2_up.launch.py

# 4. Launch on the robot (LiDAR driver)
ros2 launch ydlidar_ros_driver ydlidar_launch.py
