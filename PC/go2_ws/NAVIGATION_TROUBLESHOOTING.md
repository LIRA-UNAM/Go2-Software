# Troubleshooting: Robot errático en go2_navigation

## Análisis del log

### 1. **EMCL2 no recibía el mapa** (CORREGIDO)
- **Síntoma:** `[emcl2]: Not yet received map. Therefore, MCL cannot be initiated.`
- **Causa:** El `static_map_server` publica en `/nav_map`, pero EMCL2 estaba suscrito a `/map`.
- **Corrección:** Remap de EMCL2 `map` → `/nav_map` en `go2_navigation.launch.xml`.

### 2. **RViz2 fallaba al arrancar** (CORREGIDO)
- **Síntoma:** `Couldn't parse log level: '--log-level rviz2:=WARN'`
- **Causa:** ROS2 Foxy no soporta esa sintaxis de `--log-level` para rviz2.
- **Corrección:** Eliminado el argumento `--log-level rviz2:=WARN` del nodo rviz2.

### 3. **Conflicto TF: dogbase vs RF2O** (CORREGIDO)
- **Síntoma:** El robot "salta" de posición o se mueve errático.
- **Causa:** Tanto `dogbase` (en el robot) como `rf2o_laser_odometry` (en el PC) publican `odom` → `base_link`. Las dos fuentes compiten y el TF "parpadea".
- **Solución:** Al lanzar el robot **para navegación**, usa:
  ```bash
  ros2 launch ydlidar_ros2_driver ydlidar_launch.py publish_tf:=false
  ```
  Así dogbase no publica TF y solo RF2O lo hace.

### 4. **TF base_link ↔ laser_frame**
- **Síntoma:** `Could not find a connection between 'base_link' and 'laser_frame' because they are not part of the same tree`
- **Causa:** El YDLidar publica `/scan` con `frame_id: laser_frame`, pero ese frame debe estar conectado a `base_link` en el árbol TF.
- **Solución:** Ejecutar **siempre** `surge_et_ambula go2_up.launch.py` **antes** de `go2_navigation`. El go2_up incluye `static_tf_pub_laser` (Head_upper → laser_frame) y `robot_state_publisher`, que completan el árbol TF.

### 5. **Orden de arranque**

**Opcion A: AMCL (recomendado, evita odom_filter y drift de RF2O)**
1. **Robot:** `ros2 launch ydlidar_ros2_driver ydlidar_launch.py`
2. **PC:** `ros2 launch surge_et_ambula go2_up.launch.py map_name:=my_house`
3. **PC:** `ros2 launch navigation_start go2_navigation_amcl.launch.xml`

**Opcion B: RF2O + EMCL2 (requiere odom_filter compilado)**
1. **Robot:** `ros2 launch ydlidar_ros2_driver ydlidar_launch.py publish_tf:=false`
2. **PC:** `ros2 launch surge_et_ambula go2_up.launch.py map_name:=my_house`
3. **PC:** `ros2 launch navigation_start go2_navigation.launch.xml`

### 6. **EMCL2: frame_id vacío en /scan**
- **Síntoma:** `Failed to compute lidar pose, skipping scan (Invalid argument "" passed to lookupTransform argument source_frame - in tf2 frame_ids cannot be empty)`
- **Causa:** El scan llega con `frame_id` vacío; EMCL2 no puede calcular la pose del lidar.
- **Solución:** En `emcl2_node.cpp` función `getLidarPose`, añadir fallback:
  ```cpp
  std::string frame_id = scan_frame_id_.empty() ? "laser_frame" : scan_frame_id_;
  ident.header.frame_id = frame_id;
  ```
  O verificar que el YDLidar tenga `frame_id: laser_frame` en su params (TG.yaml).

### 7. **¿Usas publish_tf:=false en el robot?**
Si el robot sigue "loco", confirma que lanzas:
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py publish_tf:=false
```
Sin esto, dogbase y RF2O compiten publicando odom→base_link.

### 8. **Deriva de RF2O cuando el robot esta parado** (MITIGADO)
- **Sintoma:** El robot "salta" en RViz aunque este quieto.
- **Causa:** RF2O integra ruido de scan-matching como movimiento real.
- **Mitigacion aplicada:**
  - **odom_filter**: nodo con zona muerta (1.5 cm, 1.1 grados). Movimientos menores se ignoran.
  - RF2O publica en /odom_rf2o_raw sin TF; el filtro publica /odom_rf2o + TF.
  - EMCL2 con mayor ruido de odom y RF2O freq=10 Hz.

### 9. **Error "odom_filter.py not found" (ALTERNATIVA: usar AMCL)**
- **Sintoma:** `executable 'odom_filter.py' not found on the libexec directory`
- **Causa:** El paquete potential_fields no se compilo tras anadir odom_filter.py.
- **Solucion rapida:** Usar la via AMCL que no requiere odom_filter:
  ```bash
  ros2 launch navigation_start go2_navigation_amcl.launch.xml
  ```
  Requiere go2_up corriendo antes (map_server + AMCL). Usa odometria del robot en lugar de RF2O.

### 10. **Modelo del robot sin 2 patas en RViz**
- **Sintoma:** Las patas traseras (RL, RR) no se ven en el modelo 3D.
- **Causa:** `joint_state_relay` filtraba motores con `mode==0`, generando menos posiciones que nombres de joints. Las patas traseras quedaban sin datos validos.
- **Correccion:** El relay ahora usa indice fijo y asigna 0.0 a motores deshabilitados, publicando siempre 12 posiciones para las 4 patas.

### 11. **Pose inicial en EMCL2**
Si el robot no localiza bien, define la pose inicial en el mapa. En `go2_navigation.launch.xml` puedes añadir parámetros al nodo emcl2:
```xml
<param name="initial_pose_x" value="0.0"/>
<param name="initial_pose_y" value="0.0"/>
<param name="initial_pose_a" value="0.0"/>
```
Ajusta los valores según la posición real del robot en el mapa.
