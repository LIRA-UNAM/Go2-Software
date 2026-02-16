# Configuración del workspace del PC

El workspace del PC incluye paquetes del Robot necesarios para que toda la cadena TF (árbol de transformadas) corra en el PC:

- **unitree_go**: Symlink a `Robot/go2_robot_ws/src/unitree_go` (mensajes LowState, etc.)
- **go2_description**: Copia de `Robot/go2_robot_ws/src/go2_description` (joint_state_relay, URDF)

## Compilación

```bash
source /opt/ros/foxy/setup.bash
cd /path/to/Go2-Software/PC/go2_ws
colcon build --symlink-install
source install/setup.bash
```

El orden de compilación se resuelve automáticamente: `unitree_go` (mensajes) se compila antes que `go2_description` (depende de unitree_go).

## Si unitree_go no está en el workspace

Si el symlink de unitree_go se perdió, créalo:

```bash
cd /path/to/Go2-Software/PC/go2_ws/src
ln -sf ../../../Robot/go2_robot_ws/src/unitree_go unitree_go
```

## Si go2_description está desactualizado

Sincronizar desde el Robot:

```bash
cd /path/to/Go2-Software
rsync -av --exclude='__pycache__' --exclude='*.pyc' \
  Robot/go2_robot_ws/src/go2_description/ \
  PC/go2_ws/src/go2_description/
```
