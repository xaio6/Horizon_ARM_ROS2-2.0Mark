import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def _read_text(package_name, *relative_path):
    path = os.path.join(get_package_share_directory(package_name), *relative_path)
    with open(path, "r", encoding="utf-8") as text_file:
        return text_file.read()


def _read_yaml(package_name, *relative_path):
    path = os.path.join(get_package_share_directory(package_name), *relative_path)
    with open(path, "r", encoding="utf-8") as yaml_file:
        return yaml.safe_load(yaml_file)


def generate_launch_description():
    sdk_root = LaunchConfiguration("sdk_root")
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    hardware_enabled = LaunchConfiguration("hardware_enabled")
    enable_joint_gui = LaunchConfiguration("enable_joint_gui")
    enable_pose_consistency_monitor = LaunchConfiguration("enable_pose_consistency_monitor")
    enable_safety_objects = LaunchConfiguration("enable_safety_objects")
    enable_table_collision = LaunchConfiguration("enable_table_collision")
    enable_workspace_bounds = LaunchConfiguration("enable_workspace_bounds")
    enable_tool_collision = LaunchConfiguration("enable_tool_collision")
    safety_frame = LaunchConfiguration("safety_frame")
    floor_z = LaunchConfiguration("floor_z")
    table_z = LaunchConfiguration("table_z")
    driver_config = PathJoinSubstitution([
        FindPackageShare("horizon_arm_driver"),
        "config",
        "horizon_arm_v2.yaml",
    ])
    rviz_config = PathJoinSubstitution([
        FindPackageShare("horizon_arm_moveit_config"),
        "rviz",
        "moveit_real.rviz",
    ])
    horizon_config_dir = PathJoinSubstitution([
        FindPackageShare("horizon_arm_moveit_config"),
        "config",
    ])

    robot_description = {
        "robot_description": _read_text(
            "horizon_arm_description", "urdf", "horizon_arm.urdf"
        )
    }
    robot_description_semantic = {
        "robot_description_semantic": _read_text(
            "horizon_arm_moveit_config", "config", "horizon_arm.srdf"
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": _read_yaml(
            "horizon_arm_moveit_config", "config", "kinematics.yaml"
        )
    }
    robot_description_planning = {
        "robot_description_planning": _read_yaml(
            "horizon_arm_moveit_config", "config", "joint_limits.yaml"
        )
    }
    planning_pipeline = PathJoinSubstitution([
        FindPackageShare("horizon_arm_moveit_config"),
        "config",
        "move_group_ompl.yaml",
    ])
    moveit_controllers = _read_yaml(
        "horizon_arm_moveit_config", "config", "moveit_controllers.yaml"
    )
    trajectory_execution = _read_yaml(
        "horizon_arm_moveit_config", "config", "trajectory_execution.yaml"
    )
    tool_workspace_config = _read_yaml(
        "horizon_arm_moveit_config", "config", "tool_workspace_config.yaml"
    )
    tcp_offset_mm = [
        float(value) for value in tool_workspace_config.get("tcp_offset_mm", [0.0, 0.0, 0.0])
    ]
    workspace_limits_mm = tool_workspace_config.get("workspace_limits_mm", {})
    tool_collision = tool_workspace_config.get("tool_collision", {})
    planning_scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    driver = Node(
        package="horizon_arm_driver",
        executable="horizon_arm_driver",
        name="horizon_arm_driver",
        output="screen",
        parameters=[
            driver_config,
            {
                "hardware_enabled": ParameterValue(hardware_enabled, value_type=bool),
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
        parameters=[robot_description],
        remappings=[("/joint_states", "/horizon_arm/joint_states")],
    )

    world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_link",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        remappings=[("/joint_states", "/horizon_arm/joint_states")],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline,
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor,
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
        ],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="horizon_arm_target_joint_state_publisher",
        output="screen",
        condition=IfCondition(enable_joint_gui),
        parameters=[robot_description],
        remappings=[("/joint_states", "/horizon_arm/target_joint_states")],
    )

    target_bridge = Node(
        package="horizon_arm_control",
        executable="target_joint_state_bridge",
        name="horizon_arm_target_joint_state_bridge",
        output="screen",
        condition=IfCondition(enable_joint_gui),
        parameters=[
            {
                "target_topic": "/horizon_arm/target_joint_states",
                "action_name": "/horizon_arm_controller/follow_joint_trajectory",
                "send_rate_hz": 4.0,
                "trajectory_duration_sec": 0.8,
                "min_delta_rad": 0.003,
                "target_settle_time_sec": 0.0,
                "result_cooldown_sec": 0.08,
                "require_target_change_after_start": True,
                "current_state_topic": "/horizon_arm/joint_states",
                "hold_unchanged_joints_at_current": True,
            }
        ],
    )

    safety_objects = Node(
        package="horizon_arm_bringup",
        executable="planning_scene_safety_objects.py",
        name="horizon_arm_planning_scene_safety_objects",
        output="screen",
        condition=IfCondition(enable_safety_objects),
        parameters=[
            {
                "world_frame": safety_frame,
                "floor_enabled": True,
                "floor_z": floor_z,
                "floor_thickness": 0.06,
                "floor_size_x": 2.0,
                "floor_size_y": 2.0,
                "table_enabled": ParameterValue(enable_table_collision, value_type=bool),
                "table_z": table_z,
                "table_thickness": 0.03,
                "table_size_x": 1.2,
                "table_size_y": 1.2,
                "workspace_box_enabled": ParameterValue(enable_workspace_bounds, value_type=bool),
                "workspace_min_z": float(workspace_limits_mm.get("min_z", -200.0)) / 1000.0,
                "workspace_max_z": float(workspace_limits_mm.get("max_z", 600.0)) / 1000.0,
                "workspace_half_x": float(workspace_limits_mm.get("max_radius", 600.0)) / 1000.0,
                "workspace_half_y": float(workspace_limits_mm.get("max_radius", 600.0)) / 1000.0,
                "tool_collision_enabled": ParameterValue(enable_tool_collision, value_type=bool),
                "tool_collision_link": "tool0",
                "tool_collision_size": [
                    float(value) / 1000.0
                    for value in tool_collision.get("size_mm", [40.0, 40.0, 120.0])
                ],
                "tool_collision_center_offset": [
                    float(value) / 1000.0
                    for value in tool_collision.get("center_offset_mm", [0.0, 15.0, 60.0])
                ],
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

    return LaunchDescription([
        DeclareLaunchArgument("sdk_root", default_value=""),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("hardware_enabled", default_value="true"),
        DeclareLaunchArgument("enable_joint_gui", default_value="false"),
        DeclareLaunchArgument("enable_pose_consistency_monitor", default_value="true"),
        DeclareLaunchArgument("enable_safety_objects", default_value="false"),
        DeclareLaunchArgument("enable_table_collision", default_value="false"),
        DeclareLaunchArgument("enable_workspace_bounds", default_value="false"),
        DeclareLaunchArgument("enable_tool_collision", default_value="false"),
        DeclareLaunchArgument("safety_frame", default_value="base_link"),
        DeclareLaunchArgument("floor_z", default_value="-0.03"),
        DeclareLaunchArgument("table_z", default_value="-0.015"),
        SetEnvironmentVariable("HORIZONARM_CONFIG_DIR", horizon_config_dir),
        driver,
        state_waiter,
        RegisterEventHandler(
            OnProcessExit(
                target_action=state_waiter,
                on_exit=[
                    robot_state_publisher,
                    world_to_base,
                    move_group,
                    joint_state_publisher_gui,
                    target_bridge,
                    safety_objects,
                    pose_consistency_monitor,
                    rviz,
                ],
            )
        ),
    ])
