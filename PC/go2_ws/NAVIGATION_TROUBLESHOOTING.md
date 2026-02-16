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
- **Causa:** Tanto `dogbase` como RF2O/odom_filter publicaban `odom` → `base_link`.
- **Solución:** El robot usa `publish_tf:=false` fijo. dogbase publica `/utlidar/robot_odom` (topic). El PC publica el TF (odom_publisher o odom_filter).

### 4. **TF base_link ↔ laser_frame**
- **Síntoma:** `Could not find a connection between 'base_link' and 'laser_frame' because they are not part of the same tree`
- **Causa:** El YDLidar publica `/scan` con `frame_id: laser_frame`, pero ese frame debe estar conectado a `base_link` en el árbol TF.
- **Solución:** Los launches `go2_navigation_amcl` y `go2_navigation_amcl_rf2o` incluyen go2_up (robot_state_publisher + static_tf_pub_laser). No hay que lanzar go2_up por separado.

### 5. **Arquitectura de launches** (unificada)
- **go2_up:** Siempre igual: robot (TF, joystick, cámara). Sin odom ni mapas. Incluido por los launches de navegación.
- **Robot (ydlidar_launch):** Siempre igual, sin argumentos. dogbase publica /utlidar/robot_odom (no TF).
- **go2_navigation_amcl:** Incluye go2_up + odom_publisher + map_server + AMCL (odom del robot).
- **go2_navigation_amcl_rf2o:** Incluye go2_up + RF2O + odom_filter + map_server + AMCL (odom desde /scan).

### 6. **Orden de arranque** (unificado: robot sin argumentos)

**Robot:** Siempre igual, sin argumentos:
```bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

**Opcion A: AMCL con odom del robot**
```bash
ros2 launch navigation_start go2_navigation_amcl.launch.xml map_name:=my_house
```
Incluye go2_up + odom_publisher (dogbase publica /utlidar/robot_odom, odom_publisher hace TF).

**Opcion B: AMCL + RF2O** (odom desde /scan YDLidar)
```bash
ros2 launch navigation_start go2_navigation_amcl_rf2o.launch.xml map_name:=my_house
```
Incluye go2_up + RF2O + odom_filter.

**Opcion C: RF2O + EMCL2** (requiere odom_filter compilado)
1. **Robot:** `ros2 launch ydlidar_ros2_driver ydlidar_launch.py`
2. **PC:** `ros2 launch navigation_start go2_navigation.launch.xml map_name:=my_house`

### 7. **RF2O: frame_id vacio o QoS incompatible con /scan**
- **Sintoma:** RF2O no recibe scans o falla lookupTransform con frame_id vacio.
- **Causa:** Incompatibilidad QoS (YDLidar usa SensorDataQoS) o frame_id no propagado.
- **Correccion aplicada:** RF2O usa SensorDataQoS y parametro laser_frame_id como fallback si scan.header.frame_id esta vacio.

### 8. **EMCL2: frame_id vacío en /scan**
- **Síntoma:** `Failed to compute lidar pose, skipping scan (Invalid argument "" passed to lookupTransform argument source_frame - in tf2 frame_ids cannot be empty)`
- **Causa:** El scan llega con `frame_id` vacío; EMCL2 no puede calcular la pose del lidar.
- **Solución:** En `emcl2_node.cpp` función `getLidarPose`, añadir fallback:
  ```cpp
  std::string frame_id = scan_frame_id_.empty() ? "laser_frame" : scan_frame_id_;
  ident.header.frame_id = frame_id;
  ```
  O verificar que el YDLidar tenga `frame_id: laser_frame` en su params (TG.yaml).

### 9. **Conflicto TF: dogbase vs RF2O** (CORREGIDO)
El robot usa publish_tf:=false fijo. No hay que pasar argumentos. dogbase publica /utlidar/robot_odom; el PC publica el TF según el launch de navegación.

### 10. **RF2O: laser y odometria desincronizados / robot se pierde al girar**
- **Sintoma:** Con RF2O el laser "gira mas" que la odometria; el robot se pierde mucho al rotar.
- **Causas:** RF2O subestima la rotacion (comun en scan-matching 2D); desfase de timestamps TF/scan; zona muerta del odom_filter retrasa actualizaciones de rotacion.
- **Correcciones aplicadas:**
  1. **odom_filter:** Usa `get_clock().now()` para el TF (evita que el modelo no se vea si el scan tiene stamp del robot).
  2. **AMCL (go2_navigation_amcl_rf2o):** alpha1/alpha2 mas altos (0.5) para que AMCL confie mas en el laser que en la odom para rotacion; transform_tolerance 1.0 s.
  3. **odom_filter:** angular_threshold reducido a 0.01 rad (~0.6°) para rotaciones mas responsivas.
- **Si persiste:** Verificar TF laser_frame→base_link; RF2O es sensible a la orientacion del lidar.

### 11. **Mapa y augmented_map desincronizados en RViz**
- **Sintoma:** El mapa estatico y el augmented_map aparecen en posiciones distintas.
- **Correcciones aplicadas:** RViz muestra ambos superpuestos (Static Map en /map, Augmented Map en /augmented_map). map_augmenter fuerza header.frame_id="map" al publicar. Los map yaml tienen frame_id: map.
- **Verificacion:** Ambos deben usar Fixed Frame "map" y tener el mismo origin en los yaml.

### 12. **Deriva de RF2O cuando el robot esta parado** (MITIGADO)
- **Sintoma:** El robot "salta" en RViz aunque este quieto.
- **Causa:** RF2O integra ruido de scan-matching como movimiento real.
- **Mitigacion aplicada:**
  - **odom_filter**: nodo con zona muerta (1.5 cm, 1.1 grados). Movimientos menores se ignoran.
  - RF2O publica en /odom_rf2o_raw sin TF; el filtro publica /odom_rf2o + TF.
  - EMCL2 con mayor ruido de odom y RF2O freq=10 Hz.

### 13. **Error "odom_filter not found"**
- **Sintoma:** `executable 'odom_filter' not found`
- **Causa:** El paquete my_go2_launch no se compiló tras añadir odom_filter.
- **Solución:** `colcon build --packages-select my_go2_launch` o usar `go2_navigation_amcl.launch.xml` (no requiere odom_filter).

### 14. **Modelo del robot sin 2 patas en RViz**
- **Sintoma:** Las patas traseras (RL, RR) no se ven en el modelo 3D.
- **Causa:** `joint_state_relay` filtraba motores con `mode==0`, generando menos posiciones que nombres de joints. Las patas traseras quedaban sin datos validos.
- **Correccion:** El relay ahora usa indice fijo y asigna 0.0 a motores deshabilitados, publicando siempre 12 posiciones para las 4 patas.

### 15. **Pose inicial por defecto (sin 2D Pose Estimate)**
AMCL usa pose inicial por parametros (initial_pose.x, initial_pose.y, initial_pose.yaw) con valor por defecto (0, 0, 0). Para cambiar:
```bash
ros2 launch navigation_start go2_navigation_amcl_rf2o.launch.xml initial_pose_x:=-2.0 initial_pose_y:=-0.5 initial_pose_yaw:=0.0
```
Ajusta segun la posicion real del robot en el mapa. Si la pose es incorrecta, usa "2D Pose Estimate" en RViz para corregir.

### 16. **odom_filter publica TF inicial al arranque** (CORREGIDO)
- **Problema:** RF2O tarda ~2 scans (~200 ms) en publicar; durante ese tiempo no existía `odom→base_link` y AMCL fallaba.
- **Corrección:** `odom_filter` publica TF `odom→base_link` en (0,0,0) cada 100 ms hasta que llega el primer mensaje de RF2O. Así AMCL tiene el árbol TF completo desde el arranque.

### 17. **"Invalid frame ID 'map' passed to canTransform - frame does not exist"**
- **Sintoma:** Al arrancar, mvn_pln, potential_fields, simple_move reportan que el frame "map" no existe.
- **Causa:** AMCL publica `map→odom` solo tras activarse y localizar. Los nodos de navegación arrancan antes y buscan el transform hasta que AMCL lo publica.
- **Comportamiento:** Son avisos **transitorios al inicio**; suelen desaparecer en ~1 s cuando AMCL publica `map→odom`. Si persisten, verifica que go2_up esté incluido y que AMCL reciba el mapa.

### 18. **No se ven los frames `odom` ni `map` en RViz**
- **Síntoma:** El árbol TF no muestra `map` ni `odom`; Fixed Frame en RViz da error.
- **Causas posibles:**
  1. **AMCL sin pose inicial:** AMCL no publica `map→odom` hasta que se establezca la pose inicial. En RViz, usa el botón **"2D Pose Estimate"** y haz clic en el mapa donde está el robot.
  2. **odom_filter no corre:** Si usas RF2O, el `odom_filter` debe publicar `odom→base_link`. Verifica que `go2_navigation_amcl_rf2o.launch.xml` lance correctamente el nodo (usa `my_go2_launch` con `exec="odom_filter"`).
  3. **Orden de arranque:** Robot (ydlidar_launch) y luego `go2_navigation_amcl_rf2o` (incluye go2_up).
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

### 19. **AMCL no usa el mapa aumentado**
AMCL se suscribe solo a `/map` (mapa estático del static_map_server). El mapa aumentado `/augmented_map` (franja amarilla, óvalo rosa en RViz) es solo para planificación de rutas y no afecta la localización. AMCL usa exclusivamente las celdas ocupadas (negro) del mapa estático.

### 20. **Pose inicial en EMCL2**
Si el robot no localiza bien, define la pose inicial en el mapa. En `go2_navigation.launch.xml` puedes añadir parámetros al nodo emcl2:
```xml
<param name="initial_pose_x" value="0.0"/>
<param name="initial_pose_y" value="0.0"/>
<param name="initial_pose_a" value="0.0"/>
```
Ajusta los valores según la posición real del robot en el mapa.

### 21. **AMCL: "Failed to transform initial pose in time (extrapolation into the future)"**
- **Síntoma:** Al usar "2D Pose Estimate" en RViz, AMCL muestra este aviso aunque la pose se aplica.
- **Causa:** El TF `odom→base_link` usa `get_clock().now()`; el mensaje de RViz puede tener timestamp ligeramente futuro respecto al último TF.
- **Nota:** Usar `msg.header.stamp` del robot puede romper la visualización en RViz si el reloj del robot y del PC no están sincronizados (NTP). Se mantiene `get_clock().now()`. El odom_publisher publica TF inicial cada 100 ms hasta el primer mensaje del robot.

### 22. **RF2O: modelo no visible en RViz (AMCL sí funciona)** (CORREGIDO)
- **Síntoma:** Con `go2_navigation_amcl` se ve el robot en RViz; con `go2_navigation_amcl_rf2o` no.
- **Causa:** `odom_filter` usaba `msg.header.stamp` para el TF. RF2O recibe `/scan` del robot; el stamp del odom puede heredar el reloj del robot. Si robot y PC no están sincronizados, el TF tiene timestamps incompatibles y RViz no puede transformar el modelo.
- **Diferencia clave:** AMCL usa `odom_publisher` (odom del robot) → TF con `get_clock().now()`. RF2O usa `odom_filter` (odom de RF2O desde /scan) → antes usaba `msg.header.stamp`.
- **Corrección:** `odom_filter` ahora usa `get_clock().now()` para el TF, igual que `odom_publisher`.

### 23. **Localización OK con joystick, se pierde al navegar a goal**
- **Síntoma:** Con movimientos pequeños (control manual) la localización va bien; al enviar un goal y navegar autónomamente, la bola rosa crece y la localización se vuelve inestable.
- **Causas probables:** En navegación autónoma el robot se mueve más rápido; la odom de encoders tiene más deriva a velocidades altas (wheel slip). AMCL confía demasiado en una odom ruidosa.
- **Soluciones:**
  1. **Confiar más en el láser:** Subir `amcl_alpha1`–`amcl_alpha4` (ver AMCL_CALIBRATION.md, ejemplo "Navegación autónoma").
  2. **Reducir velocidad:** Editar `go2_navigation_amcl.launch.xml` y bajar `max_linear_speed` (p. ej. de 0.8 a 0.5) en el nodo simple_move.
  3. **Sincronización relojes:** Si robot y PC tienen relojes distintos, considerar NTP. Tanto `odom_publisher` como `odom_filter` usan `get_clock().now()` para el TF (evita que el modelo no se vea en RViz).
