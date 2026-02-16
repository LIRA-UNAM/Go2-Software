# Prueba de cámara Go2 (img_publisher)

## Copiar código al robot (desde PC)

```bash
scp -r /home/joel/UNAM/LIRA/unitree/Go2/Go2-Software/Robot unitree@192.168.123.18:/home/unitree/
```

## Dependencias (sin internet)

El `img_publisher` usa PyGObject/GStreamer porque OpenCV no tiene soporte GStreamer en el robot.

**Importante:** El Go2 es **arm64**. Descarga la versión arm64, no amd64.

**En un PC con internet** (Ubuntu amd64):

Opción A – apt (añade arm64 y descarga):
```bash
cd /tmp
sudo dpkg --add-architecture arm64
sudo apt-get update
apt-get download python3-gst-1.0:arm64
scp /tmp/python3-gst-1.0_*_arm64.deb unitree@192.168.123.18:/home/unitree/
```

Opción B – Descarga directa (robot = Ubuntu 20 Focal, PC = Ubuntu 22):
```bash
cd /tmp
# Robot usa Ubuntu 20.04 (Focal) - descargar versión 1.16.2:
wget http://ports.ubuntu.com/pool/universe/g/gst-python1.0/python3-gst-1.0_1.16.2-2_arm64.deb
scp python3-gst-1.0_1.16.2-2_arm64.deb unitree@192.168.123.18:/home/unitree/
```

**En el robot** (Ubuntu 20):
```bash
sudo dpkg -i /home/unitree/python3-gst-1.0_1.16.2-2_arm64.deb
```

Si falta `gir1.2-gstreamer-1.0`, descárgalo igual (arm64 Focal) desde ports.ubuntu.com.

## Probar que GStreamer recibe el stream

```bash
gst-launch-1.0 udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! \
  application/x-rtp, media=video, encoding-name=H264 ! \
  rtph264depay ! h264parse ! fakesink sync=false
```

Si ves "Setting pipeline to PLAYING" y no hay errores, la cámara emite correctamente.

## Probar img_publisher

1. **Compilar** (en el robot):
   ```bash
   cd ~/Robot/go2_robot_ws
   colcon build --packages-select my_go2_launch
   source install/setup.bash
   ```

2. **Lanzar con cámara**:
   ```bash
   ros2 launch surge_et_ambula go2_up.launch.py launch_camera:=true
   ```

3. **Verificar publicación** (en otra terminal, robot o PC):
   ```bash
   ros2 topic hz /camera/image_raw
   ```
   Deberías ver ~30 Hz si hay comunicación.

4. **Ver un frame** (opcional):
   ```bash
   ros2 topic echo /camera/image_raw --once
   ```

## Si falla

- **"No se pudo cargar GStreamer"**: Instala `python3-gst-1.0` (ver arriba).
- **"no element avdec_h264"** / **"libgomp cannot allocate memory in static TLS block"**: El launch ya incluye `LD_PRELOAD` para libgomp. Si ejecutas el nodo manualmente: `LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1 ros2 run my_go2_launch img_publisher`.
- **"No se pudo abrir el stream"**: El robot debe estar en pie (Stand). Prueba `gst-launch` primero.
- **Interfaz incorrecta**: Si `eth0` no es la correcta, pasa `multicast_iface` en el launch o usa `ip route get 230.1.1.1` para ver cuál usar.
