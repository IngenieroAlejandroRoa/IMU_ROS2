import math
from typing import Tuple

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


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


class ArtificialHorizonNode(Node):
    def __init__(self) -> None:
        super().__init__("artificial_horizon_node")

        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("euler_topic", "imu/euler_deg")
        self.declare_parameter("marker_topic", "imu/artificial_horizon")
        self.declare_parameter("pitch_offset_per_deg", 0.008)
        self.declare_parameter("instrument_width", 1.2)
        self.declare_parameter("instrument_height", 0.8)
        self.declare_parameter("instrument_x", 1.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.euler_topic = str(self.get_parameter("euler_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.pitch_offset_per_deg = float(self.get_parameter("pitch_offset_per_deg").value)
        self.instrument_width = float(self.get_parameter("instrument_width").value)
        self.instrument_height = float(self.get_parameter("instrument_height").value)
        self.instrument_x = float(self.get_parameter("instrument_x").value)

        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.euler_sub = self.create_subscription(Vector3Stamped, self.euler_topic, self._on_euler, 20)
        self.get_logger().info(f"Horizonte artificial listo. Escuchando {self.euler_topic}")

    def _on_euler(self, msg: Vector3Stamped) -> None:
        roll_deg = msg.vector.x
        pitch_deg = msg.vector.y

        roll_qx, roll_qy, roll_qz, roll_qw = euler_to_quaternion(-roll_deg, 0.0, 0.0)
        pitch_offset = -pitch_deg * self.pitch_offset_per_deg

        now = self.get_clock().now().to_msg()
        half_h = self.instrument_height * 0.5

        sky = Marker()
        sky.header.stamp = now
        sky.header.frame_id = self.frame_id
        sky.ns = "artificial_horizon"
        sky.id = 0
        sky.type = Marker.CUBE
        sky.action = Marker.ADD
        sky.pose.position.x = self.instrument_x
        sky.pose.position.y = pitch_offset + (half_h * 0.5)
        sky.pose.orientation.x = roll_qx
        sky.pose.orientation.y = roll_qy
        sky.pose.orientation.z = roll_qz
        sky.pose.orientation.w = roll_qw
        sky.scale.x = 0.03
        sky.scale.y = self.instrument_width
        sky.scale.z = half_h
        sky.color.r = 0.22
        sky.color.g = 0.50
        sky.color.b = 0.95
        sky.color.a = 0.95

        ground = Marker()
        ground.header.stamp = now
        ground.header.frame_id = self.frame_id
        ground.ns = "artificial_horizon"
        ground.id = 1
        ground.type = Marker.CUBE
        ground.action = Marker.ADD
        ground.pose.position.x = self.instrument_x
        ground.pose.position.y = pitch_offset - (half_h * 0.5)
        ground.pose.orientation.x = roll_qx
        ground.pose.orientation.y = roll_qy
        ground.pose.orientation.z = roll_qz
        ground.pose.orientation.w = roll_qw
        ground.scale.x = 0.03
        ground.scale.y = self.instrument_width
        ground.scale.z = half_h
        ground.color.r = 0.52
        ground.color.g = 0.33
        ground.color.b = 0.16
        ground.color.a = 0.95

        horizon_line = Marker()
        horizon_line.header.stamp = now
        horizon_line.header.frame_id = self.frame_id
        horizon_line.ns = "artificial_horizon"
        horizon_line.id = 2
        horizon_line.type = Marker.CUBE
        horizon_line.action = Marker.ADD
        horizon_line.pose.position.x = self.instrument_x + 0.002
        horizon_line.pose.position.y = pitch_offset
        horizon_line.pose.orientation.x = roll_qx
        horizon_line.pose.orientation.y = roll_qy
        horizon_line.pose.orientation.z = roll_qz
        horizon_line.pose.orientation.w = roll_qw
        horizon_line.scale.x = 0.031
        horizon_line.scale.y = self.instrument_width
        horizon_line.scale.z = 0.01
        horizon_line.color.r = 1.0
        horizon_line.color.g = 1.0
        horizon_line.color.b = 1.0
        horizon_line.color.a = 1.0

        markers = MarkerArray()
        markers.markers = [sky, ground, horizon_line]
        self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArtificialHorizonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
