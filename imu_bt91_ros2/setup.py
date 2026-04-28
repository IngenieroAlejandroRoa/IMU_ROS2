from glob import glob
from setuptools import find_packages, setup

package_name = "imu_bt91_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "bleak"],
    zip_safe=True,
    maintainer="ingeniero",
    maintainer_email="ingeniero@example.com",
    description="ROS 2 driver and 3D visualization marker for BT91 IMU over BLE.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ble_imu_node = imu_bt91_ros2.ble_imu_node:main",
            "artificial_horizon_node = imu_bt91_ros2.artificial_horizon_node:main",
            "artificial_horizon_gui_node = imu_bt91_ros2.artificial_horizon_gui_node:main",
        ],
    },
)
