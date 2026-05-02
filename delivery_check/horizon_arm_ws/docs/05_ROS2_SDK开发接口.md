# ROS2 SDK 开发接口

更新时间：2026-05-01

## 启动入口

```bash
ros2 launch horizon_arm_bringup sdk_real.launch.py \
  sdk_root:=/home/yun/delivery_check/Horizon_Arm2.0_sdk_linux \
  arm_port:=/dev/ttyACM0 \
  arm_baudrate:=115200 \
  io_port:=/dev/ttyUSB0 \
  io_baudrate:=115200
```

## ROS2 接口

### 1. 基础运动

- Action: `/horizon_arm_controller/follow_joint_trajectory`

### 2. 机械臂管理

- Service: `/horizon_arm/enable`
- Service: `/horizon_arm/disable`
- Service: `/horizon_arm/emergency_stop`
- Topic: `/horizon_arm/status`
- Topic: `/horizon_arm/joint_states`

### 3. 数字输出

- Service: `/horizon_arm/set_digital_output`

```text
uint8 channel
bool state
---
bool success
string message
```

### 4. 夹爪

- Service: `/horizon_arm/set_gripper_state`

```text
bool open
int32 current_ma
---
bool success
string message
```

说明：

- `open=true`：张开
- `open=false`：闭合

### 5. 统一指令

- Action: `/horizon_arm/run_instruction`

支持文本指令：

- `enable`
- `disable`
- `estop`
- `preset:home_position`
- `set_do:0=1`
- `gripper:open`
- `gripper:close`
- `gripper:open=1200`
- `gripper:close=1200`

支持 JSON 指令：

```json
{"command":"preset","name":"home_position"}
{"command":"move_joints_deg","joints":[0,10,0,0,0,0],"duration":2.0}
{"command":"move_joints_deg","joints":[[0,0,0,0,0,0],[20,0,0,0,0,0]],"duration":3.0}
{"command":"move_joints_rad","joints":[0.0,0.2,0.0,0.0,0.0,0.0],"duration":2.0}
{"command":"set_digital_output","channel":0,"state":true}
{"command":"set_gripper_state","open":true,"current_ma":1200}
```

### 6. 视觉抓取

- Service: `/horizon_arm/visual_grasp`

```text
bool dry_run
bool use_bbox
float32 u
float32 v
float32 x1
float32 y1
float32 x2
float32 y2
---
bool success
string message
```

说明：

- `dry_run=true`：仅做健康检查
- `use_bbox=false`：按像素点抓取
- `use_bbox=true`：按框选区域抓取

### 7. 跟随抓取控制

- Service: `/horizon_arm/follow_grasp_control`

```text
string command
string target_class
float32 conf_thres
float32 interval_sec
---
bool success
bool running
string message
```

支持命令：

- `health`
- `status`
- `start`
- `stop`

### 8. Joycon 控制

- Service: `/horizon_arm/joycon_control`

```text
string command
---
bool success
bool running
string message
```

支持命令：

- `status`
- `connect`
- `disconnect`
- `start`
- `stop`

### 9. 具身智能指令

- Service: `/horizon_arm/embodied_instruction`

```text
string command
bool stream
---
bool success
string message
string result_json
```

说明：

- `command=health`：仅检查 wrapper 和 SDK 能力
- 其他自然语言指令依赖相关 AI 配置

## Python SDK

包内提供 `HorizonArmRosSdk`，可直接在二次开发脚本中调用。

```python
import rclpy
from rclpy.node import Node

from horizon_arm_control import HorizonArmRosSdk


class Demo(Node):
    def __init__(self):
        super().__init__("horizon_arm_demo")
        self.sdk = HorizonArmRosSdk(self)


def main():
    rclpy.init()
    node = Demo()
    try:
        if not node.sdk.wait_until_ready(
            include_instruction=True,
            include_digital_output=True,
            include_gripper=True,
            include_extended_wrappers=True,
        ):
            raise RuntimeError("ROS2 SDK services are not ready")

        print(node.sdk.enable())
        print(node.sdk.execute_preset("home_position"))
        print(node.sdk.move_joints_deg([0, 20, 0, 0, 0, 0], duration_sec=2.0))
        print(node.sdk.set_digital_output(0, True))
        print(node.sdk.close_gripper())
        print(node.sdk.open_gripper())
        print(node.sdk.visual_grasp_health())
        print(node.sdk.follow_grasp_status())
        print(node.sdk.joycon_status())
        print(node.sdk.embodied_health())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## 说明

- 视觉抓取 / 跟随默认提供的是统一 ROS2 入口和健康检查路径
- Joycon 默认提供的是状态和控制入口
- 具身智能默认提供的是服务桥接入口
