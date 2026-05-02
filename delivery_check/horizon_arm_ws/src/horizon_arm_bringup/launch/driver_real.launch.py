from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sdk_root = LaunchConfiguration("sdk_root")
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    config = PathJoinSubstitution([
        FindPackageShare("horizon_arm_driver"),
        "config",
        "horizon_arm_v2.yaml",
    ])

    driver = Node(
        package="horizon_arm_driver",
        executable="horizon_arm_driver",
        name="horizon_arm_driver",
        output="screen",
        parameters=[
            config,
            {
                "hardware_enabled": True,
                "sdk_root": sdk_root,
                "port": port,
                "baudrate": baudrate,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("sdk_root", default_value=""),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        driver,
    ])

