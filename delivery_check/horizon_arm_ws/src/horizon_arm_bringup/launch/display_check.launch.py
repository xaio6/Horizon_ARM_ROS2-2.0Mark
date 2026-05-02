import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _robot_description():
    urdf_path = os.path.join(
        get_package_share_directory("horizon_arm_description"),
        "urdf",
        "horizon_arm.urdf",
    )
    with open(urdf_path, "r", encoding="utf-8") as urdf_file:
        return urdf_file.read()


def generate_launch_description():
    robot_description = _robot_description()
    rviz_config = PathJoinSubstitution([
        FindPackageShare("horizon_arm_description"),
        "rviz",
        "horizon_arm.rviz",
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="horizon_arm_model_joint_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
