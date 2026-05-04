# horizon_arm_interfaces

`horizon_arm_interfaces` 定义 Horizon Arm 2.0 在 ROS2 中使用的自定义消息、服务和动作接口。构建后，这些接口可在命令行、Python 和 C++ 节点中直接使用。

字段级说明见：

```text
delivery_check/horizon_arm_ws/docs/05_ROS2_SDK开发接口.md
```

## 消息

| 文件 | 用途 |
|---|---|
| `msg/ArmStatus.msg` | 机械臂连接状态、电机使能状态、6 轴角度、速度和告警 |

`ArmStatus` 字段：

| 字段 | 含义 |
|---|---|
| `stamp` | 状态时间戳 |
| `hardware_connected` | 硬件是否连接 |
| `motors_enabled` | 电机是否使能 |
| `joint_position_deg` | 6 轴角度，单位度 |
| `joint_velocity_deg_s` | 6 轴速度，单位度/秒 |
| `warnings` | 告警列表 |

## 服务

| 文件 | ROS2 服务名 | 用途 |
|---|---|---|
| `SetDigitalOutput.srv` | `/horizon_arm/set_digital_output` | 控制外部 IO 输出 |
| `SetGripperState.srv` | `/horizon_arm/set_gripper_state` | 控制夹爪张开或闭合 |
| `VisualGrasp.srv` | `/horizon_arm/visual_grasp` | 基础视觉抓取 |
| `VisualGraspEx.srv` | `/horizon_arm/visual_grasp_ex` | 增强视觉抓取、点击抓取、bbox 抓取、dry-run |
| `VisionConfig.srv` | `/horizon_arm/vision_config` | 设置或查询视觉检测参数 |
| `PickHSV.srv` | `/horizon_arm/pick_hsv` | 图像像素 HSV 取样 |
| `DetectTarget.srv` | `/horizon_arm/detect_target` | HSV/YOLO 目标检测 |
| `FollowGraspControl.srv` | `/horizon_arm/follow_grasp_control` | 基础跟随抓取控制 |
| `FollowTarget.srv` | `/horizon_arm/follow_target` | 增强目标跟随 |
| `JoyconControl.srv` | `/horizon_arm/joycon_control` | Joy-Con 基础控制 |
| `JoyconAdvancedControl.srv` | `/horizon_arm/joycon_advanced_control` | Joy-Con 高级控制和参数配置 |
| `TeachJog.srv` | `/horizon_arm/teach_jog` | 示教点动和 dry-run 规划 |
| `EmbodiedInstruction.srv` | `/horizon_arm/embodied_instruction` | 兼容版具身智能入口 |
| `EmbodiedCommand.srv` | `/horizon_arm/embodied_command` | 扩展具身智能命令入口 |

## 动作

| 文件 | ROS2 Action 名 | 用途 |
|---|---|---|
| `RunInstruction.action` | `/horizon_arm/run_instruction` | 高层统一动作入口 |
| `TeachingProgram.action` | `/horizon_arm/teaching_program` | 示教程序保存、加载、校验、运行 |

## 命令行查看

构建并 source 后可查看接口：

```bash
ros2 interface show horizon_arm_interfaces/msg/ArmStatus
ros2 interface show horizon_arm_interfaces/srv/VisualGraspEx
ros2 interface show horizon_arm_interfaces/action/RunInstruction
```

查看服务是否已启动：

```bash
ros2 service list | grep horizon_arm
ros2 action list | grep horizon_arm
```

## 示例

夹爪张开：

```bash
ros2 service call /horizon_arm/set_gripper_state horizon_arm_interfaces/srv/SetGripperState \
'{open: true, current_ma: 1200}'
```

高层关节运动：

```bash
ros2 action send_goal /horizon_arm/run_instruction horizon_arm_interfaces/action/RunInstruction \
'{instruction: "{\"command\":\"move_joints_deg\",\"joints\":[0,5,0,0,0,0],\"duration\":2.0}"}'
```
