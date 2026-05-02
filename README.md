# Horizon Arm ROS2 v2

Horizon Arm 2.0 的 Ubuntu / ROS2 工作空间。

## 快速开始

```bash
source /opt/ros/jazzy/setup.bash
source /home/yun/delivery_check/horizon_arm_ws/install/setup.bash

ros2 launch horizon_arm_bringup sdk_real.launch.py \
  sdk_root:=/home/yun/delivery_check/Horizon_Arm2.0_sdk_linux \
  arm_port:=/dev/ttyACM0 \
  arm_baudrate:=115200 \
  io_port:=/dev/ttyUSB0 \
  io_baudrate:=115200
```

```bash
ros2 run horizon_arm_control system_check --ros-args \
  -p sdk_root:=/home/yun/delivery_check/Horizon_Arm2.0_sdk_linux
```

## 文档

- [部署说明](./delivery_check/horizon_arm_ws/docs/部署说明.md)
- [ROS2 SDK 开发接口](.delivery_check/horizon_arm_ws//docs/05_ROS2_SDK开发接口.md)
