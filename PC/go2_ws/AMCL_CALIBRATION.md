# Calibración AMCL - Navegación Go2

Parámetros ajustables desde la línea de comandos para calibrar la localización cuando el robot se pierde.

---

## go2_navigation_amcl (odom del robot)

**Nota:** AMCL usa solo el mapa estático `/map` para localización. El mapa aumentado `/augmented_map` (franja amarilla, óvalo rosa) es solo para planificación y no afecta a AMCL.

### Comando base

```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
ros2 launch navigation_start go2_navigation_amcl.launch.xml map_name:=my_house
```

### Comando completo con todos los parámetros AMCL (defaults tipo slam_toolbox)

```bash
ros2 launch navigation_start go2_navigation_amcl.launch.xml \
  map_name:=my_house \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0 \
  amcl_alpha1:=0.15 amcl_alpha2:=0.15 amcl_alpha3:=0.12 amcl_alpha4:=0.12 amcl_alpha5:=0.05 \
  amcl_recovery_alpha_fast:=0.0 amcl_recovery_alpha_slow:=0.0 \
  amcl_transform_tolerance:=1.0 \
  amcl_min_particles:=500 amcl_max_particles:=2000 amcl_pf_err:=0.05 \
  amcl_max_beams:=60 \
  amcl_sigma_hit:=0.2 amcl_laser_likelihood_max_dist:=2.0 \
  amcl_update_min_a:=0.1 amcl_update_min_d:=0.1
```

---

## go2_navigation_amcl_rf2o (odom desde láser)

### Comando base

```bash
ros2 launch navigation_start go2_navigation_amcl_rf2o.launch.xml map_name:=my_house
```

### Comando completo con todos los parámetros (AMCL + RF2O + odom_filter)

```bash
ros2 launch navigation_start go2_navigation_amcl_rf2o.launch.xml \
  map_name:=my_house \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0 \
  rf2o_freq:=10.0 odom_filter_linear_threshold:=0.015 odom_filter_angular_threshold:=0.01 \
  amcl_alpha1:=0.5 amcl_alpha2:=0.5 amcl_alpha3:=0.3 amcl_alpha4:=0.3 amcl_alpha5:=0.1 \
  amcl_recovery_alpha_fast:=0.1 amcl_recovery_alpha_slow:=0.001 \
  amcl_transform_tolerance:=1.0 \
  amcl_min_particles:=1000 amcl_max_particles:=3000 amcl_pf_err:=0.1 \
  amcl_max_beams:=90 \
  amcl_sigma_hit:=0.2 amcl_laser_likelihood_max_dist:=2.0 \
  amcl_update_min_a:=0.2 amcl_update_min_d:=0.25
```

## Parámetros RF2O y odom_filter (solo go2_navigation_amcl_rf2o)

| Parámetro | Default | Descripción | Si el robot se pierde... |
|-----------|---------|-------------|---------------------------|
| **rf2o_freq** | 10.0 | Frecuencia RF2O (Hz) | Probar 8–12 |
| **odom_filter_linear_threshold** | 0.015 | Zona muerta lineal (m) | Bajar si el robot "salta" al moverse |
| **odom_filter_angular_threshold** | 0.01 | Zona muerta angular (rad) | Bajar a 0.005 si RF2O subestima rotación |

## Parámetros AMCL y guía de ajuste

| Parámetro | Default (odom) | Default (RF2O) | Descripción | Si el robot se pierde... |
|-----------|----------------|----------------|-------------|---------------------------|
| **amcl_alpha1** | 0.15 | 0.5 | Ruido odom: rotación por rotación (tipo slam_toolbox) | Subir a 0.3–0.5 (confiar más en el láser) |
| **amcl_alpha2** | 0.15 | 0.5 | Ruido odom: rotación por traslación | Subir si la orientación se desvía al avanzar |
| **amcl_alpha3** | 0.12 | 0.3 | Ruido odom: traslación por traslación | Subir si la odom lineal es mala |
| **amcl_alpha4** | 0.12 | 0.3 | Ruido odom: traslación por rotación | Subir si al girar la posición se desvía |
| **amcl_alpha5** | 0.05 | 0.1 | Ruido odom omnidireccional | Normalmente no cambiar |
| **amcl_recovery_alpha_fast** | 0.0 | 0.1 | Recuperación rápida (0=desactivada) | Activar 0.1 si se pierde mucho |
| **amcl_recovery_alpha_slow** | 0.0 | 0.001 | Recuperación lenta | Activar 0.001 si se pierde mucho |
| **amcl_transform_tolerance** | 1.0 | 1.0 | Validez TF map→odom (s) | Subir si hay desfases de tiempo |
| **amcl_min_particles** | 500 | 1000 | Partículas mínimas | Subir a 1000–1500 para más cobertura |
| **amcl_max_particles** | 2000 | 3000 | Partículas máximas | Subir a 3000–5000 si sigue perdiéndose |
| **amcl_pf_err** | 0.05 | 0.1 | Umbral error particle filter | Probar 0.05–0.1 |
| **amcl_max_beams** | 60 | 90 | Beams del láser | Más = más preciso pero más lento (60–120) |
| **amcl_sigma_hit** | 0.2 | 0.2 | Desviación modelo gaussiano | Más bajo = más estricto |
| **amcl_laser_likelihood_max_dist** | 2.0 | 2.0 | Dist. max inflación obstáculos (m) | Normalmente no cambiar |
| **amcl_update_min_a** | 0.1 | 0.2 | Rotación mínima para actualizar (rad) | Bajar a 0.05 para más reactivo |
| **amcl_update_min_d** | 0.1 | 0.25 | Traslación mínima para actualizar (m) | Bajar a 0.05 para más reactivo |

## Ejemplos de pruebas

**Más confianza en el láser (odom ruidosa):**
```bash
ros2 launch navigation_start go2_navigation_amcl.launch.xml map_name:=my_house \
  amcl_alpha1:=0.5 amcl_alpha2:=0.5 amcl_alpha3:=0.3 amcl_alpha4:=0.3
```

**RF2O: más confianza en el láser (RF2O subestima rotación):**
```bash
ros2 launch navigation_start go2_navigation_amcl_rf2o.launch.xml map_name:=my_house \
  amcl_alpha1:=0.6 amcl_alpha2:=0.6 odom_filter_angular_threshold:=0.005
```

**Más partículas para recuperarse mejor:**
```bash
ros2 launch navigation_start go2_navigation_amcl.launch.xml map_name:=my_house \
  amcl_min_particles:=1500 amcl_max_particles:=5000
```

**Actualizaciones más frecuentes:**
```bash
ros2 launch navigation_start go2_navigation_amcl.launch.xml map_name:=my_house \
  amcl_update_min_a:=0.1 amcl_update_min_d:=0.15
```

**Localización OK con joystick, se pierde en navegación autónoma:**
```bash
# Confiar más en el láser (odom ruidosa a velocidades altas)
ros2 launch navigation_start go2_navigation_amcl.launch.xml map_name:=my_house \
  amcl_alpha1:=0.35 amcl_alpha2:=0.35 amcl_alpha3:=0.25 amcl_alpha4:=0.25

# Opcional: reducir velocidad de navegación
ros2 launch navigation_start go2_navigation_amcl.launch.xml map_name:=my_house \
  max_linear_speed:=0.5 amcl_alpha1:=0.3 amcl_alpha2:=0.3
```
