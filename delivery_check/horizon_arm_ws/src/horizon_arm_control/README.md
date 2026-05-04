# horizon_arm_control

`horizon_arm_control` 是 Horizon Arm 2.0 的 ROS 2 高层控制包，负责把底层驱动、Linux SDK 和自定义接口组织成开发者可直接调用的节点与 Python SDK。

本文命令使用 `HORIZON_SDK_ROOT`、`HORIZON_ARM_PORT`、`HORIZON_IO_PORT` 等变量。变量含义和设置方法见工作空间根目录的 `docs/部署说明.md` 第 0 节。

## 包内内容

- `run_instruction_server`
  统一高层动作入口，覆盖使能、失能、急停、预设动作、关节运动、夹爪、数字输出。
- `digital_output_server`
  把 IO SDK 封装成 `/horizon_arm/set_digital_output` 服务。
- `visual_grasp_server`
  提供基础视觉抓取、增强视觉抓取、视觉配置、HSV 取样、目标检测。
- `follow_grasp_server`
  提供基础跟随抓取控制和增强版 `/horizon_arm/follow_target` 服务。
- `joycon_server`
  提供简版 Joy-Con 控制和完整高级参数控制。
- `teaching_server`
  提供示教点动服务和示教程序 Action。
- `embodied_server`
  提供自然语言任务编排、函数查询、历史记录和紧急停止。
- `HorizonArmRosSdk`
  面向教学脚本和二次开发节点的阻塞式 Python SDK。

## 快速使用

先启动整套 bringup：

```bash
ros2 launch horizon_arm_bringup sdk_real.launch.py \
  sdk_root:="${HORIZON_SDK_ROOT}" \
  arm_port:="${HORIZON_ARM_PORT}" \
  arm_baudrate:=115200 \
  io_port:="${HORIZON_IO_PORT}" \
  io_baudrate:=115200
```

Python SDK 最小示例：

```python
import rclpy
from rclpy.node import Node
from horizon_arm_control import HorizonArmRosSdk

rclpy.init()
node = Node("horizon_arm_demo")
sdk = HorizonArmRosSdk(node)

sdk.wait_until_ready(
    include_instruction=True,
    include_digital_output=True,
    include_gripper=True,
    include_extended_wrappers=True,
)
sdk.enable()
sdk.move_joints_deg([0, -20, 45, 0, 30, 0], duration_sec=2.0)
sdk.close_gripper(current_ma=1200)
```

## 重点开发接口

| 分类 | Python SDK 方法 | 对应 ROS2 接口 | 输入 | 输出 |
|---|---|---|---|---|
| 基础状态 | `enable()`、`disable()`、`emergency_stop()` | `/horizon_arm/enable`、`/horizon_arm/disable`、`/horizon_arm/emergency_stop` | 无输入或超时参数 | `success/message` |
| 关节运动 | `move_joints_deg()`、`move_joints_rad()` | `/horizon_arm_controller/follow_joint_trajectory` | 6 轴目标角、运动时长 | `success/error_code/error_string` |
| 高层指令 | `run_instruction()`、`execute_preset()` | `/horizon_arm/run_instruction` | 短文本或 JSON 指令 | `success/message` |
| IO | `set_digital_output()` | `/horizon_arm/set_digital_output` | 通道号、开关状态 | `success/message` |
| 夹爪 | `open_gripper()`、`close_gripper()`、`set_gripper_state()` | `/horizon_arm/set_gripper_state` | 开合状态、电流 mA | `success/message` |
| 视觉 | `configure_vision()`、`pick_hsv()`、`detect_target()` | `/horizon_arm/vision_config`、`/horizon_arm/pick_hsv`、`/horizon_arm/detect_target` | pipeline、HSV、像素点、阈值 | JSON 结果 |
| 抓取 | `visual_grasp()`、`visual_grasp_ex()` | `/horizon_arm/visual_grasp`、`/horizon_arm/visual_grasp_ex` | 点、bbox、dry_run、深度参数 | JSON 结果 |
| 跟随 | `follow_grasp_control()`、`follow_target()` | `/horizon_arm/follow_grasp_control`、`/horizon_arm/follow_target` | command、mode、目标类别、options_json | JSON 状态 |
| Joy-Con | `joycon_control()`、`joycon_advanced()` | `/horizon_arm/joycon_control`、`/horizon_arm/joycon_advanced_control` | command、模式、速度和姿态参数 | 状态 JSON |
| 示教 | `teach_jog()`、`teach_jog_joint()`、`run_teaching_program()` | `/horizon_arm/teach_jog`、`/horizon_arm/teaching_program` | 点动参数、程序 JSON、dry_run | 目标角或程序结果 |
| 具身智能 | `embodied_instruction()`、`embodied_command()` | `/horizon_arm/embodied_instruction`、`/horizon_arm/embodied_command` | health/functions/actions/run 等 | JSON 结果 |

字段级说明和每个接口的命令行例子见 `docs/05_ROS2_SDK开发接口.md`。

## 开发提示

- `follow_target` 的手动框选模式支持通过 `options_json` 传入 `x1/y1/x2/y2`，先调用 `set_target`，再调用 `start` 或 `resume`。
- `follow_target` 的 `options_json` 也可传入 `hsv_h_min/hsv_h_max/hsv_s_min/hsv_s_max/hsv_v_min/hsv_v_max`，用于 HSV 跟随。
- `teach_jog` 中的 `base_translate`、`tool_translate`、`base_rotate`、`tool_rotate`、`cartesian_move` 现在会基于当前 `joint_states` 做正逆运动学规划，并返回目标关节角；`dry_run:=false` 时会直接复用轨迹 Action 执行。

## 示例脚本

源码目录提供以下示例：

- `examples/basic_motion_demo.py`
  SDK 就绪检查与基础关节运动示例，默认只检查连接，传 `--execute` 后才执行运动。
- `examples/vision_pipeline_demo.py`
  演示 `VisionConfig -> DetectTarget -> VisualGraspEx(dry_run)` 的完整调用链。
- `examples/teaching_demo.py`
  演示示教状态查询、关节点动 dry-run、示教程序校验。
- `examples/embodied_demo.py`
  演示函数查询、动作查询以及可选的自然语言任务调用。

安装后这些示例会一起安装到：

```bash
$(ros2 pkg prefix horizon_arm_control)/share/horizon_arm_control/examples
```

## 相关文档

- 工作区总览：`delivery_check/horizon_arm_ws/README.md`
- 部署说明：`delivery_check/horizon_arm_ws/docs/部署说明.md`
- SDK 接口速查：`delivery_check/horizon_arm_ws/docs/05_ROS2_SDK开发接口.md`
- 核心功能教学开发指南：`delivery_check/horizon_arm_ws/docs/Horizon_ARM_ROS2_2.0核心功能教学开发指南.md`

