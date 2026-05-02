import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    sdk_root = LaunchConfiguration("sdk_root")
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    enable_pose_consistency_monitor = LaunchConfiguration("enable_pose_consistency_monitor")
    driver_config = PathJoinSubstitution([
        FindPackageShare("horizon_arm_driver"),
        "config",
        "horizon_arm_v2.yaml",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("horizon_arm_description"),
        "rviz",
        "horizon_arm.rviz",
    ])
    horizon_config_dir = PathJoinSubstitution([
        FindPackageShare("horizon_arm_moveit_config"),
        "config",
    ])
    tool_workspace_config = os.path.join(
        get_package_share_directory("horizon_arm_moveit_config"),
        "config",
        "tool_workspace_config.yaml",
    )
    with open(tool_workspace_config, "r", encoding="utf-8") as handle:
        import yaml
        tool_workspace = yaml.safe_load(handle) or {}
    tcp_offset_mm = [float(value) for value in tool_workspace.get("tcp_offset_mm", [0.0, 0.0, 0.0])]
    robot_description = _robot_description()

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

    state_waiter = Node(
        package="horizon_arm_bringup",
        executable="wait_for_joint_state_stability.py",
        name="horizon_arm_joint_state_stability_waiter",
        output="screen",
        parameters=[{"joint_state_topic": "/horizon_arm/joint_states"}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        remappings=[("/joint_states", "/horizon_arm/joint_states")],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="horizon_arm_target_joint_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        remappings=[("/joint_states", "/horizon_arm/target_joint_states")],
    )

    target_bridge = Node(
        package="horizon_arm_control",
        executable="target_joint_state_bridge",
        name="horizon_arm_target_joint_state_bridge",
        output="screen",
        parameters=[
            {
                "target_topic": "/horizon_arm/target_joint_states",
                "action_name": "/horizon_arm_controller/follow_joint_trajectory",
                "send_rate_hz": 12.0,
                "trajectory_duration_sec": 0.35,
                "min_delta_rad": 0.004,
                "target_settle_time_sec": 0.0,
                "result_cooldown_sec": 0.04,
                "require_target_change_after_start": True,
                "current_state_topic": "/horizon_arm/joint_states",
                "hold_unchanged_joints_at_current": True,
            }
        ],
    )

    pose_consistency_monitor = Node(
        package="horizon_arm_bringup",
        executable="rviz_pose_consistency_monitor.py",
        name="horizon_arm_rviz_pose_consistency_monitor",
        output="screen",
        condition=IfCondition(enable_pose_consistency_monitor),
        parameters=[{"tcp_offset_mm": tcp_offset_mm, "joint_state_topic": "/horizon_arm/joint_states"}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        DeclareLaunchArgument("sdk_root", default_value=""),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("enable_pose_consistency_monitor", default_value="true"),
        SetEnvironmentVariable("HORIZONARM_CONFIG_DIR", horizon_config_dir),
        driver,
        state_waiter,
        RegisterEventHandler(
            OnProcessExit(
                target_action=state_waiter,
                on_exit=[
                    robot_state_publisher,
                    joint_state_publisher_gui,
                    target_bridge,
                    pose_consistency_monitor,
                    rviz,
                ],
            )
        ),
    ])
