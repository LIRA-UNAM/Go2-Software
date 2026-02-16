# Go2-Software
Software for the Unitree Go2 quadruped robot 

This repository contains the files required to control the **GO2** robot with ROS 2 Foxy.  
Execution is divided into two parts: **Local PC** (visualization and navigation) and **Robot** (sensors and drivers). 
For more details refer to Robot/Mauricio/Go2 Quick Setup.pdf

---

## 🚀 Execution Steps

```bash
# Configure a wired connection with IP 192.168.123.X with x != 18, netmask 255.255.255.0 and Gateway 192.168.123.1
# 0. Clone this repository
# 1. Connect to the robot via SSH
ssh unitree@192.168.123.18
# password: 123
source /opt/ros/foxy/setup.bash   # load ROS Foxy on the robot

# 2. Copy the Robot folder to the robot (from the local PC, outside SSH)
scp -r /path/to/repo/Robot unitree@192.168.123.18:/home/unitree/

# 3. Build both workspaces

## On the local PC:
Execute:
./path/to/repo/PC/Setup.sh
source /opt/ros/foxy/setup.bash
cd path/to/repo/PC/go2_ws
# El workspace incluye unitree_go (symlink) y go2_description para el árbol TF en PC
colcon build --symlink-install
source install/setup.bash

## On the robot:
ssh unitree@192.168.123.18 #Only if you are disconnected 
source /opt/ros/foxy/setup.bash
cd ~/go2_robot_ws   # o cd ~/Robot/go2_robot_ws si copiaste la carpeta Robot
colcon list         # verifica surge_et_ambula, go2_description, my_go2_launch, joystick_teleop, ydlidar, go2_demo, unitree_*
colcon build --symlink-install
source install/setup.bash

## build and install ydlidar sdk
cd src/
cd YDLidar-SDK
mkdir build
cd build
cmake ..
make
sudo make install



Note: It's necessary change the network interface inside gstreamer_image_publisher.py node.


# 4. Launch on the robot (ydlidar + dogbase + TF + joystick + cámara + mapas)
ros2 launch surge_et_ambula go2_up.launch.py
# Con otro mapa: ros2 launch surge_et_ambula go2_up.launch.py map_name:=lab

# 5. Navegación en el PC (RF2O + AMCL + planificación)
source install/setup.bash   # ¡Importante! Cargar el workspace del PC
ros2 launch navigation_start go2_navigation_amcl_rf2o.launch.xml
# Alternativa con odom del robot: go2_navigation_amcl.launch.xml
# Alternativa con EMCL2: go2_navigation.launch.xml
# Opcional: usar otro mapa (ej. lab)
ros2 launch navigation_start go2_navigation_amcl_rf2o.launch.xml map_name:=lab

# Nota: El topic /scan debe tener frame_id (ej. laser_frame). Verifica que el YDLidar
# en el robot tenga frame_id configurado en params (TG.yaml).

