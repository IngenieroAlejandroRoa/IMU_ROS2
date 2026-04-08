from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("imu_bt91_ros2")
    rviz_config = os.path.join(pkg_share, "rviz", "imu_3d.rviz")

    address_arg = DeclareLaunchArgument(
        "address",
        default_value="C7:B5:46:C9:36:9D",
        description="Direccion MAC de la IMU BT91",
    )
    frame_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="imu_link",
        description="Frame publicado por la IMU",
    )

    imu_node = Node(
        package="imu_bt91_ros2",
        executable="ble_imu_node",
        name="bt91_imu_node",
        parameters=[
            {
                "address": LaunchConfiguration("address"),
                "frame_id": LaunchConfiguration("frame_id"),
            }
        ],
        output="screen",
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", LaunchConfiguration("frame_id")],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="imu_rviz",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([address_arg, frame_arg, imu_node, static_tf_node, rviz_node])
