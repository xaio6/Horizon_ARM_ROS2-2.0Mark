from setuptools import find_packages, setup

package_name = "horizon_arm_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/horizon_arm_v2.yaml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Horizon Arm Team",
    maintainer_email="dev@horizon-arm.local",
    description="ROS2 Jazzy hardware driver for Horizon Arm 2.0 over UCP / OmniCAN.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "horizon_arm_driver = horizon_arm_driver.driver_node:main",
        ],
    },
)

