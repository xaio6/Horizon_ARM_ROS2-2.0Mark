from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sdk_root = LaunchConfiguration("sdk_root")
    report_dir = LaunchConfiguration("report_dir")
    real_hardware = LaunchConfiguration("real_hardware")
    arm_port = LaunchConfiguration("arm_port")
    arm_baudrate = LaunchConfiguration("arm_baudrate")
    io_port = LaunchConfiguration("io_port")
    io_baudrate = LaunchConfiguration("io_baudrate")
    io_timeout_sec = LaunchConfiguration("io_timeout_sec")
    preset_config_path = LaunchConfiguration("preset_config_path")
    preset_name = LaunchConfiguration("preset_name")
    live_step_delay_sec = LaunchConfiguration("live_step_delay_sec")
    camera_id = LaunchConfiguration("camera_id")
    camera_hardware_available = LaunchConfiguration("camera_hardware_available")
    io_hardware_available = LaunchConfiguration("io_hardware_available")

    horizon_config_dir = PathJoinSubstitution(
        [FindPackageShare("horizon_arm_moveit_config"), "config"]
    )
    driver_config = PathJoinSubstitution(
        [FindPackageShare("horizon_arm_driver"), "config", "horizon_arm_v2.yaml"]
    )

    common_parameters = [{"sdk_root": sdk_root}]

    driver = Node(
        package="horizon_arm_driver",
        executable="horizon_arm_driver",
        name="horizon_arm_driver",
        output="screen",
        parameters=[
            driver_config,
            {
                "hardware_enabled": ParameterValue(real_hardware, value_type=bool),
                "sdk_root": sdk_root,
                "port": arm_port,
                "baudrate": ParameterValue(arm_baudrate, value_type=int),
            },
        ],
    )

    system_check = Node(
        package="horizon_arm_control",
        executable="system_check",
        name="horizon_arm_system_check",
        output="screen",
        parameters=[
            {
                "sdk_root": sdk_root,
                "report_dir": report_dir,
                "acceptance_profile": "full_acceptance",
                "allow_hardware_side_effects": ParameterValue(real_hardware, value_type=bool),
                "preset_name": preset_name,
                "live_step_delay_sec": ParameterValue(live_step_delay_sec, value_type=float),
                "camera_id": ParameterValue(camera_id, value_type=int),
                "camera_hardware_available": ParameterValue(camera_hardware_available, value_type=bool),
                "io_hardware_available": ParameterValue(io_hardware_available, value_type=bool),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("sdk_root", default_value=""),
            DeclareLaunchArgument("report_dir", default_value=""),
            DeclareLaunchArgument("real_hardware", default_value="false"),
            DeclareLaunchArgument("arm_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("arm_baudrate", default_value="115200"),
            DeclareLaunchArgument("io_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("io_baudrate", default_value="115200"),
            DeclareLaunchArgument("io_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("preset_config_path", default_value=""),
            DeclareLaunchArgument("preset_name", default_value=""),
            DeclareLaunchArgument("live_step_delay_sec", default_value="2.0"),
            DeclareLaunchArgument("camera_id", default_value="0"),
            DeclareLaunchArgument("camera_hardware_available", default_value="true"),
            DeclareLaunchArgument("io_hardware_available", default_value="true"),
            SetEnvironmentVariable("HORIZONARM_CONFIG_DIR", horizon_config_dir),
            driver,
            Node(
                package="horizon_arm_control",
                executable="run_instruction_server",
                name="horizon_arm_run_instruction_server",
                output="screen",
                parameters=[{"preset_config_path": preset_config_path}],
            ),
            Node(
                package="horizon_arm_control",
                executable="digital_output_server",
                name="horizon_arm_digital_output_server",
                output="screen",
                parameters=[
                    {
                        "sdk_root": sdk_root,
                        "port": io_port,
                        "baudrate": ParameterValue(io_baudrate, value_type=int),
                        "timeout_sec": ParameterValue(io_timeout_sec, value_type=float),
                    }
                ],
            ),
            Node(
                package="horizon_arm_control",
                executable="visual_grasp_server",
                name="horizon_arm_visual_grasp_server",
                output="screen",
                parameters=common_parameters
                + [{"camera_id": ParameterValue(camera_id, value_type=int)}],
            ),
            Node(
                package="horizon_arm_control",
                executable="follow_grasp_server",
                name="horizon_arm_follow_grasp_server",
                output="screen",
                parameters=common_parameters
                + [{"camera_id": ParameterValue(camera_id, value_type=int)}],
            ),
            Node(
                package="horizon_arm_control",
                executable="joycon_server",
                name="horizon_arm_joycon_server",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="horizon_arm_control",
                executable="teaching_server",
                name="horizon_arm_teaching_server",
                output="screen",
                parameters=common_parameters,
            ),
            Node(
                package="horizon_arm_control",
                executable="embodied_server",
                name="horizon_arm_embodied_server",
                output="screen",
                parameters=common_parameters,
            ),
            TimerAction(period=3.0, actions=[system_check]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=system_check,
                    on_exit=[EmitEvent(event=Shutdown(reason="acceptance check completed"))],
                )
            ),
        ]
    )
