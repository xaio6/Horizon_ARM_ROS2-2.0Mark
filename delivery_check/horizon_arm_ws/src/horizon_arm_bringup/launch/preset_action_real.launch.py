import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sdk_root = LaunchConfiguration("sdk_root")
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    preset_name = LaunchConfiguration("preset_name")
    driver_config = PathJoinSubstitution([
        FindPackageShare("horizon_arm_driver"),
        "config",
        "horizon_arm_v2.yaml",
    ])
    horizon_config_dir = PathJoinSubstitution([
        FindPackageShare("horizon_arm_moveit_config"),
        "config",
    ])

    driver = Node(
        package="horizon_arm_driver",
        executable="horizon_arm_driver",
        name="horizon_arm_driver",
        output="screen",
        parameters=[
            driver_config,
            {
                "hardware_enabled": True,
                "sdk_root": sdk_root,
                "port": port,
                "baudrate": baudrate,
            },
        ],
    )

    executor = Node(
        package="horizon_arm_bringup",
        executable="execute_preset_action.py",
        name="horizon_arm_execute_preset_action",
        output="screen",
        parameters=[{"preset_name": preset_name}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("sdk_root", default_value=""),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("preset_name", default_value="home_position"),
        SetEnvironmentVariable("HORIZONARM_CONFIG_DIR", horizon_config_dir),
        driver,
        executor,
    ])
