# horizon_arm_driver

`horizon_arm_driver` 是 Horizon Arm 2.0 的 ROS 2 硬件驱动包，负责管理 UCP 通信、关节状态发布、轨迹执行以及基础使能/急停/夹爪服务。

本文命令使用 `HORIZON_SDK_ROOT`、`HORIZON_ARM_PORT` 等变量。变量含义和设置方法见工作空间根目录的 `docs/部署说明.md` 第 0 节。

## 主要职责

- 管理真实硬件连接与参数映射
- 发布 `/joint_states`、`/horizon_arm/joint_states`、`/horizon_arm/status`
- 提供 `/horizon_arm/enable`、`/horizon_arm/disable`、`/horizon_arm/emergency_stop`
- 提供 `/horizon_arm/set_gripper_state`
- 提供 `/horizon_arm_controller/follow_joint_trajectory`

## 启动方式

单独启动驱动：

```bash
ros2 run horizon_arm_driver horizon_arm_driver --ros-args \
  -p hardware_enabled:=true \
  -p sdk_root:="${HORIZON_SDK_ROOT}" \
  -p port:="${HORIZON_ARM_PORT}" \
  -p baudrate:=115200
```

更常见的方式是通过 bringup：

```bash
ros2 launch horizon_arm_bringup sdk_real.launch.py \
  sdk_root:="${HORIZON_SDK_ROOT}" \
  arm_port:="${HORIZON_ARM_PORT}" \
  arm_baudrate:=115200
```

## 关键配置

驱动默认读取：

```text
share/horizon_arm_driver/config/horizon_arm_v2.yaml
```

当前配置覆盖了以下关键参数：

- 串口与波特率
- 电机关节映射
- 减速比、方向、零点偏移
- 关节限位
- 轨迹执行容差与超时
- 夹爪电流默认值

## 相关接口

- 状态主题：`/joint_states`、`/horizon_arm/joint_states`、`/horizon_arm/status`
- 动作接口：`/horizon_arm_controller/follow_joint_trajectory`
- 服务接口：`/horizon_arm/enable`、`/horizon_arm/disable`、`/horizon_arm/emergency_stop`、`/horizon_arm/set_gripper_state`

