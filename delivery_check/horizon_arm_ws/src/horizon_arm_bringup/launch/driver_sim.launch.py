from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
            {"hardware_enabled": False},
        ],
    )

    return LaunchDescription([driver])

