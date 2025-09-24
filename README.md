# Go2-Software
Software for the quadruped robot Unitree Go2

Este repositorio contiene los archivos necesarios para controlar el robot **GO2** con ROS 2 Foxy.  
La ejecución se divide en dos partes: **PC local** (visualización y navegación) y **Robot** (sensores y drivers).

---

## 🚀 Pasos de ejecución

```bash
# 0. Conectarse al robot por SSH
ssh unitree@192.168.123.18
# password: 123
source /opt/ros/foxy/setup.bash   # cargar ROS Foxy en el robot

# 1. Copiar la carpeta Robot al robot (desde la PC local, fuera del SSH)
scp -r /ruta/al/repo/GO2/Robot unitree@192.168.123.18:/home/unitree/

# 2. Compilar ambos workspaces

## En la PC local:
source /opt/ros/foxy/setup.bash
cd ~/go2_ws
colcon build --symlink-install
source install/setup.bash

## En el robot:
ssh unitree@192.168.123.18
source /opt/ros/foxy/setup.bash
cd ~/go2_robot_ws
colcon build --symlink-install
source install/setup.bash

# 3. Lanzar en la PC (stack principal con RViz)
ros2 launch surge_et_ambula go2_up.launch.py

# 4. Lanzar en el robot (driver del LiDAR)
ros2 launch ydlidar_ros_driver ydlidar.launch.py
