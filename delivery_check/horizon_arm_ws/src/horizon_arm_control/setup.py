from setuptools import find_packages, setup

package_name = "horizon_arm_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Horizon Arm Team",
    maintainer_email="dev@horizon-arm.local",
    description="High-level ROS2 control adapters for Horizon Arm 2.0.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "target_joint_state_bridge = horizon_arm_control.target_joint_state_bridge:main",
            "digital_output_server = horizon_arm_control.digital_output_server:main",
            "visual_grasp_server = horizon_arm_control.visual_grasp_server:main",
            "follow_grasp_server = horizon_arm_control.follow_grasp_server:main",
            "joycon_server = horizon_arm_control.joycon_server:main",
            "embodied_server = horizon_arm_control.embodied_server:main",
            "run_instruction_server = horizon_arm_control.run_instruction_server:main",
            "run_instruction_client = horizon_arm_control.run_instruction_client:main",
            "system_check = horizon_arm_control.system_check:main",
        ],
    },
)
