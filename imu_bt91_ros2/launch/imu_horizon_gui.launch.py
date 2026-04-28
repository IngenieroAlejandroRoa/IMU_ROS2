from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
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

    horizon_gui_node = Node(
        package="imu_bt91_ros2",
        executable="artificial_horizon_gui_node",
        name="artificial_horizon_gui_node",
        output="screen",
    )

    return LaunchDescription([address_arg, frame_arg, imu_node, horizon_gui_node])
