# IMU BT91 en ROS 2

Este repositorio incluye un paquete ROS 2 para leer una IMU BT91 por Bluetooth Low Energy y visualizar su orientacion en 3D con RViz2.

## Estructura

- `imu_bt91/`: scripts originales en Python (modo antiguo con `imu_gui.py`).
- `imu_bt91_ros2/`: paquete ROS 2 (`ament_python`) con nodo BLE + visualizacion 3D.

## Requisitos

- ROS 2 (Humble/Jazzy o compatible con `rclpy`)
- Python 3
- `bleak`

```bash
pip install bleak
```

## Visualizacion 3D en vivo (ROS 2 + RViz2)

Ejecuta exactamente estos comandos:

```bash
cd /home/ingeniero/Desktop/IMU_ROS2
colcon build --packages-select imu_bt91_ros2
source install/setup.bash
ros2 launch imu_bt91_ros2 imu_visualization.launch.py
```

Con otra MAC:

```bash
ros2 launch imu_bt91_ros2 imu_visualization.launch.py address:=AA:BB:CC:DD:EE:FF
```

### Opcion 2: Nodo y RViz por separado (manual)

Terminal 1:

```bash
cd /home/ingeniero/Desktop/IMU_ROS2
source install/setup.bash
ros2 run imu_bt91_ros2 ble_imu_node
```

Terminal 2:

```bash
cd /home/ingeniero/Desktop/IMU_ROS2
source install/setup.bash
rviz2
```

En RViz2 configura:
1. `Global Options -> Fixed Frame`: `map`
2. `Add -> Marker` y topic: `/imu/marker`
3. (Opcional) `Add -> Axes` con `Reference Frame = imu_link`

## Verificar que la IMU este publicando

```bash
ros2 topic hz /imu/data_raw
ros2 topic echo /imu/euler_deg
```

Si `/imu/euler_deg` cambia al mover la IMU, la lectura BLE esta correcta.

## Troubleshooting rapido

- **Warning `No tf data. Frame [imu_link] doesn't exist`**: usa el launch de este repo (`imu_visualization.launch.py`), que ya publica un TF estatico `map -> imu_link`.
- **Marker con `Error subscribing: empty topic name`**: en el display Marker selecciona explicitamente el topic `/imu/marker`.

## Topicos publicados

- `/imu/data_raw` (`sensor_msgs/msg/Imu`)
- `/imu/euler_deg` (`geometry_msgs/msg/Vector3Stamped`)
- `/imu/marker` (`visualization_msgs/msg/Marker`)

## Modo antiguo (sin ROS 2)

Si quieres volver al modo anterior:

```bash
cd /home/ingeniero/Desktop/IMU_ROS2/imu_bt91
python3 imu_gui.py
```
