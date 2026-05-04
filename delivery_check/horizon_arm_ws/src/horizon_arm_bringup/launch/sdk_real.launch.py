from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sdk_root = LaunchConfiguration("sdk_root")
    arm_port = LaunchConfiguration("arm_port")
    arm_baudrate = LaunchConfiguration("arm_baudrate")
    io_port = LaunchConfiguration("io_port")
    io_baudrate = LaunchConfiguration("io_baudrate")
    io_timeout_sec = LaunchConfiguration("io_timeout_sec")
    preset_config_path = LaunchConfiguration("preset_config_path")

    horizon_config_dir = PathJoinSubstitution(
        [FindPackageShare("horizon_arm_moveit_config"), "config"]
    )
    driver_config = PathJoinSubstitution(
        [FindPackageShare("horizon_arm_driver"), "config", "horizon_arm_v2.yaml"]
    )

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
                "port": arm_port,
                "baudrate": arm_baudrate,
            },
        ],
    )

    instruction_server = Node(
        package="horizon_arm_control",
        executable="run_instruction_server",
        name="horizon_arm_run_instruction_server",
        output="screen",
        parameters=[
            {
                "preset_config_path": preset_config_path,
            }
        ],
    )

    digital_output_server = Node(
        package="horizon_arm_control",
        executable="digital_output_server",
        name="horizon_arm_digital_output_server",
        output="screen",
        parameters=[
            {
                "sdk_root": sdk_root,
                "port": io_port,
                "baudrate": io_baudrate,
                "timeout_sec": io_timeout_sec,
            }
        ],
    )

    visual_grasp_server = Node(
        package="horizon_arm_control",
        executable="visual_grasp_server",
        name="horizon_arm_visual_grasp_server",
        output="screen",
        parameters=[{"sdk_root": sdk_root}],
    )

    follow_grasp_server = Node(
        package="horizon_arm_control",
        executable="follow_grasp_server",
        name="horizon_arm_follow_grasp_server",
        output="screen",
        parameters=[{"sdk_root": sdk_root}],
    )

    joycon_server = Node(
        package="horizon_arm_control",
        executable="joycon_server",
        name="horizon_arm_joycon_server",
        output="screen",
        parameters=[{"sdk_root": sdk_root}],
    )

    embodied_server = Node(
        package="horizon_arm_control",
        executable="embodied_server",
        name="horizon_arm_embodied_server",
        output="screen",
        parameters=[{"sdk_root": sdk_root}],
    )

    teaching_server = Node(
        package="horizon_arm_control",
        executable="teaching_server",
        name="horizon_arm_teaching_server",
        output="screen",
        parameters=[{"sdk_root": sdk_root}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("sdk_root", default_value=""),
            DeclareLaunchArgument("arm_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("arm_baudrate", default_value="115200"),
            DeclareLaunchArgument("io_port", default_value="/dev/ttyUSB1"),
            DeclareLaunchArgument("io_baudrate", default_value="115200"),
            DeclareLaunchArgument("io_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("preset_config_path", default_value=""),
            SetEnvironmentVariable("HORIZONARM_CONFIG_DIR", horizon_config_dir),
            driver,
            instruction_server,
            digital_output_server,
            visual_grasp_server,
            follow_grasp_server,
            joycon_server,
            teaching_server,
            embodied_server,
        ]
    )
