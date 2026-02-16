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

**Opcion A: AMCL con odom del robot** (UTLidar/odom_publisher)
1. **Robot:** `ros2 launch ydlidar_ros2_driver ydlidar_launch.py publish_tf:=true`
2. **PC:** `ros2 launch surge_et_ambula go2_up.launch.py map_name:=my_house`
3. **PC:** `ros2 launch navigation_start go2_navigation_amcl.launch.xml`
4. **En RViz:** Usa "2D Pose Estimate" para indicar dónde está el robot en el mapa.

**Opcion B: AMCL + RF2O** (odom desde /scan YDLidar, recomendado si UTLidar es lento)
1. **Robot:** `ros2 launch ydlidar_ros2_driver ydlidar_launch.py` (publish_tf:=false por defecto)
2. **PC:** `ros2 launch surge_et_ambula go2_up.launch.py map_name:=my_house use_rf2o_odom:=true`
3. **PC:** `ros2 launch navigation_start go2_navigation_amcl_rf2o.launch.xml`
4. **En RViz:** Usa "2D Pose Estimate" para indicar dónde está el robot en el mapa (AMCL no publica map→odom hasta entonces).

**Opcion C: RF2O + EMCL2** (requiere odom_filter compilado)
1. **Robot:** `ros2 launch ydlidar_ros2_driver ydlidar_launch.py`
2. **PC:** `ros2 launch surge_et_ambula go2_up.launch.py map_name:=my_house use_rf2o_odom:=true`
3. **PC:** `ros2 launch navigation_start go2_navigation.launch.xml`

### 6. **RF2O: frame_id vacio o QoS incompatible con /scan**
- **Sintoma:** RF2O no recibe scans o falla lookupTransform con frame_id vacio.
- **Causa:** Incompatibilidad QoS (YDLidar usa SensorDataQoS) o frame_id no propagado.
- **Correccion aplicada:** RF2O usa SensorDataQoS y parametro laser_frame_id como fallback si scan.header.frame_id esta vacio.

### 7. **EMCL2: frame_id vacío en /scan**
- **Síntoma:** `Failed to compute lidar pose, skipping scan (Invalid argument "" passed to lookupTransform argument source_frame - in tf2 frame_ids cannot be empty)`
- **Causa:** El scan llega con `frame_id` vacío; EMCL2 no puede calcular la pose del lidar.
- **Solución:** En `emcl2_node.cpp` función `getLidarPose`, añadir fallback:
  ```cpp
  std::string frame_id = scan_frame_id_.empty() ? "laser_frame" : scan_frame_id_;
  ident.header.frame_id = frame_id;
  ```
  O verificar que el YDLidar tenga `frame_id: laser_frame` en su params (TG.yaml).

### 8. **Conflicto TF: dogbase vs RF2O**
Cuando usas RF2O (go2_navigation_amcl_rf2o o go2_navigation con EMCL2), el robot debe lanzarse con publish_tf:=false (ahora es el valor por defecto). Si usas odom del robot (go2_navigation_amcl), necesitas publish_tf:=true.

### 9. **Deriva de RF2O cuando el robot esta parado** (MITIGADO)
- **Sintoma:** El robot "salta" en RViz aunque este quieto.
- **Causa:** RF2O integra ruido de scan-matching como movimiento real.
- **Mitigacion aplicada:**
  - **odom_filter**: nodo con zona muerta (1.5 cm, 1.1 grados). Movimientos menores se ignoran.
  - RF2O publica en /odom_rf2o_raw sin TF; el filtro publica /odom_rf2o + TF.
  - EMCL2 con mayor ruido de odom y RF2O freq=10 Hz.

### 10. **Error "odom_filter.py not found" (ALTERNATIVA: usar AMCL)**
- **Sintoma:** `executable 'odom_filter.py' not found on the libexec directory`
- **Causa:** El paquete potential_fields no se compilo tras anadir odom_filter.py.
- **Solucion rapida:** Usar la via AMCL que no requiere odom_filter:
  ```bash
  ros2 launch navigation_start go2_navigation_amcl.launch.xml
  ```
  Requiere go2_up corriendo antes (map_server + AMCL). Usa odometria del robot en lugar de RF2O.

### 11. **Modelo del robot sin 2 patas en RViz**
- **Sintoma:** Las patas traseras (RL, RR) no se ven en el modelo 3D.
- **Causa:** `joint_state_relay` filtraba motores con `mode==0`, generando menos posiciones que nombres de joints. Las patas traseras quedaban sin datos validos.
- **Correccion:** El relay ahora usa indice fijo y asigna 0.0 a motores deshabilitados, publicando siempre 12 posiciones para las 4 patas.

### 12. **No se ven los frames `odom` ni `map` en RViz**
- **Síntoma:** El árbol TF no muestra `map` ni `odom`; Fixed Frame en RViz da error.
- **Causas posibles:**
  1. **AMCL sin pose inicial:** AMCL no publica `map→odom` hasta que se establezca la pose inicial. En RViz, usa el botón **"2D Pose Estimate"** y haz clic en el mapa donde está el robot.
  2. **odom_filter no corre:** Si usas RF2O, el `odom_filter` debe publicar `odom→base_link`. Verifica que `go2_navigation_amcl_rf2o.launch.xml` lance correctamente el nodo (usa `my_go2_launch` con `exec="odom_filter"`).
  3. **Orden de arranque:** Primero `go2_up` con `use_rf2o_odom:=true`, luego `go2_navigation_amcl_rf2o`.
- **Verificación (Foxy):** Ejecuta en el **PC** donde corre `go2_navigation_amcl_rf2o`. Foxy no soporta `-n 1` en `topic echo`; usa `ros2 topic echo /odom_rf2o` (Ctrl+C tras ver un mensaje) o `ros2 topic hz /odom_rf2o_raw`. Para TF: `ros2 run tf2_ros tf2_echo odom base_link`.
- **RViz sin map/odom:** Si Fixed Frame da error, cambia temporalmente a `base_link` en Global Options para ver el robot mientras depuras.
- **Diagnóstico paso a paso (Foxy):** Ejecuta en el PC donde corre la navegación:
  ```bash
  ros2 node list | grep -E "odom_filter|rf2o"
  ros2 topic list | grep odom
  ros2 topic hz /odom_rf2o_raw
  ros2 topic hz /scan
  ros2 topic echo /odom_rf2o
  ```
  (Ctrl+C en `topic echo` tras ver un mensaje; Foxy no soporta `-n 1`). Si `/odom_rf2o_raw` no tiene Hz, RF2O no publica (revisa `/scan`). Si `odom_filter` no aparece en node list, el nodo no arrancó.

### 13. **Pose inicial en EMCL2**
Si el robot no localiza bien, define la pose inicial en el mapa. En `go2_navigation.launch.xml` puedes añadir parámetros al nodo emcl2:
```xml
<param name="initial_pose_x" value="0.0"/>
<param name="initial_pose_y" value="0.0"/>
<param name="initial_pose_a" value="0.0"/>
```
Ajusta los valores según la posición real del robot en el mapa.
