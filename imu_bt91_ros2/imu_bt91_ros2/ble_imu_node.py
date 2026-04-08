import asyncio
import math
import queue
import threading
from typing import Optional, Tuple

import rclpy
from bleak import BleakClient
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker

from imu_bt91_ros2.imu_parser import GRAVITY, ImuSample, parse_bt91_packet


def euler_to_quaternion(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[float, float, float, float]:
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class Bt91ImuNode(Node):
    def __init__(self) -> None:
        super().__init__("bt91_imu_node")

        self.declare_parameter("address", "C7:B5:46:C9:36:9D")
        self.declare_parameter("char_uuid", "0000ffe4-0000-1000-8000-00805f9a34fb")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("reconnect_delay_s", 2.0)

        self.address = self.get_parameter("address").value
        self.char_uuid = self.get_parameter("char_uuid").value
        self.frame_id = self.get_parameter("frame_id").value
        self.reconnect_delay_s = float(self.get_parameter("reconnect_delay_s").value)

        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 20)
        self.euler_pub = self.create_publisher(Vector3Stamped, "imu/euler_deg", 20)
        self.marker_pub = self.create_publisher(Marker, "imu/marker", 20)

        self.sample_queue: queue.Queue[ImuSample] = queue.Queue(maxsize=200)
        self.stop_event = threading.Event()
        self.ble_thread = threading.Thread(target=self._ble_runner, daemon=True)
        self.ble_thread.start()

        self.create_timer(1.0 / 50.0, self._publish_latest_sample)
        self.get_logger().info(f"BLE IMU node listo. Conectando a {self.address}")

    def _ble_runner(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._ble_task())
        finally:
            loop.close()

    async def _ble_task(self) -> None:
        while rclpy.ok() and not self.stop_event.is_set():
            try:
                async with BleakClient(self.address) as client:
                    if not client.is_connected:
                        raise ConnectionError("No se pudo establecer conexion BLE")

                    self.get_logger().info("Conectado a la IMU por BLE")

                    def on_notify(_: int, data: bytearray) -> None:
                        parsed = parse_bt91_packet(bytes(data))
                        if parsed is None:
                            return
                        try:
                            self.sample_queue.put_nowait(parsed)
                        except queue.Full:
                            pass

                    await client.start_notify(self.char_uuid, on_notify)
                    while rclpy.ok() and not self.stop_event.is_set():
                        await asyncio.sleep(0.05)
                    await client.stop_notify(self.char_uuid)

            except Exception as exc:
                self.get_logger().error(f"Error BLE: {exc}")
                if not self.stop_event.is_set():
                    await asyncio.sleep(self.reconnect_delay_s)

    def _publish_latest_sample(self) -> None:
        latest: Optional[ImuSample] = None
        while True:
            try:
                latest = self.sample_queue.get_nowait()
            except queue.Empty:
                break

        if latest is None:
            return

        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = euler_to_quaternion(latest.roll_deg, latest.pitch_deg, latest.yaw_deg)

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        imu_msg.angular_velocity.x = math.radians(latest.gx_dps)
        imu_msg.angular_velocity.y = math.radians(latest.gy_dps)
        imu_msg.angular_velocity.z = math.radians(latest.gz_dps)
        imu_msg.linear_acceleration.x = latest.ax_g * GRAVITY
        imu_msg.linear_acceleration.y = latest.ay_g * GRAVITY
        imu_msg.linear_acceleration.z = latest.az_g * GRAVITY
        self.imu_pub.publish(imu_msg)

        euler_msg = Vector3Stamped()
        euler_msg.header.stamp = now
        euler_msg.header.frame_id = self.frame_id
        euler_msg.vector.x = latest.roll_deg
        euler_msg.vector.y = latest.pitch_deg
        euler_msg.vector.z = latest.yaw_deg
        self.euler_pub.publish(euler_msg)

        marker_msg = Marker()
        marker_msg.header.stamp = now
        marker_msg.header.frame_id = self.frame_id
        marker_msg.ns = "bt91_orientation"
        marker_msg.id = 0
        marker_msg.type = Marker.ARROW
        marker_msg.action = Marker.ADD
        marker_msg.pose.orientation.x = qx
        marker_msg.pose.orientation.y = qy
        marker_msg.pose.orientation.z = qz
        marker_msg.pose.orientation.w = qw
        marker_msg.scale.x = 0.8
        marker_msg.scale.y = 0.08
        marker_msg.scale.z = 0.08
        marker_msg.color.r = 0.2
        marker_msg.color.g = 0.8
        marker_msg.color.b = 1.0
        marker_msg.color.a = 1.0
        self.marker_pub.publish(marker_msg)

    def destroy_node(self) -> bool:
        self.stop_event.set()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Bt91ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
