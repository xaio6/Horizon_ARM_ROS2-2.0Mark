# Horizon Arm ROS2 2.0

这是 Horizon Arm 2.0 的 Ubuntu / ROS2 Jazzy 交付工作空间。交付运行只依赖：

```text
${HORIZON_DELIVERY_ROOT}/
├── Horizon_Arm2.0_sdk_linux
└── horizon_arm_ws
```

## 0. 路径和串口变量

文档中的命令统一使用变量，部署到不同电脑时只需要先改这一段。

```bash
export HORIZON_DELIVERY_ROOT="${HOME}/delivery_check"
export HORIZON_WS="${HORIZON_DELIVERY_ROOT}/horizon_arm_ws"
export HORIZON_SDK_ROOT="${HORIZON_DELIVERY_ROOT}/Horizon_Arm2.0_sdk_linux"
export HORIZON_ARM_PORT="/dev/ttyACM0"
export HORIZON_IO_PORT="/dev/ttyUSB0"
```

变量含义：

| 变量 | 怎么填 |
|---|---|
| `HORIZON_DELIVERY_ROOT` | 交付根目录。`${HOME}` 是当前 Linux 用户目录，例如用户是 `yun` 时 `${HOME}` 通常是 `/home/yun`；如果你放在 `/home/test/delivery_check`，这里就填 `/home/test/delivery_check` |
| `HORIZON_WS` | ROS2 工作空间目录，通常是 `${HORIZON_DELIVERY_ROOT}/horizon_arm_ws` |
| `HORIZON_SDK_ROOT` | Linux SDK 根目录，通常是 `${HORIZON_DELIVERY_ROOT}/Horizon_Arm2.0_sdk_linux` |
| `HORIZON_ARM_PORT` | 机械臂串口，常见是 `/dev/ttyACM0`，以现场 `ls /dev/ttyACM*` 或 `/dev/serial/by-id/` 为准 |
| `HORIZON_IO_PORT` | IO 串口，常见是 `/dev/ttyUSB0`，没有 IO 时验收加 `--no-io` |

## 1. 文档入口

| 文档 | 读者 | 内容 |
|---|---|---|
| [部署说明](./delivery_check/horizon_arm_ws/docs/部署说明.md) | 现场部署、验收人员 | 环境、编译、启动、验收、故障处理 |
| [ROS2 SDK 开发接口说明](./delivery_check/horizon_arm_ws/docs/05_ROS2_SDK开发接口.md) | 二次开发人员 | topic/service/action/Python SDK 字段级说明 |
| [核心功能教学开发指南](./delivery_check/horizon_arm_ws/docs/Horizon_ARM_ROS2_2.0核心功能教学开发指南.md) | 教学、演示、联调人员 | 每个功能的输入、输出、例子和预期结果 |

包级说明：

| 包 | 说明 |
|---|---|
| [horizon_arm_bringup](./delivery_check/horizon_arm_ws/src/horizon_arm_bringup/README.md) | launch、验收脚本、启动入口 |
| [horizon_arm_control](./delivery_check/horizon_arm_ws/src/horizon_arm_control/README.md) | 高层控制服务和 Python SDK |
| [horizon_arm_driver](./delivery_check/horizon_arm_ws/src/horizon_arm_driver/README.md) | 真实硬件驱动 |
| [horizon_arm_interfaces](./delivery_check/horizon_arm_ws/src/horizon_arm_interfaces/README.md) | 自定义 msg/srv/action |

## 2. 首次编译

进入工作空间：

```bash
cd "${HORIZON_WS}"
```

编译相关包：

```bash
chmod +x src/horizon_arm_bringup/scripts/run_acceptance_check.py

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  horizon_arm_interfaces \
  horizon_arm_driver \
  horizon_arm_control \
  horizon_arm_bringup
source install/setup.bash
```

确认一条命令验收入口已安装：

```bash
ros2 pkg executables horizon_arm_bringup
```

预期看到：

```text
horizon_arm_bringup run_acceptance_check.py
```

如果没有这一行，`ros2 run horizon_arm_bringup run_acceptance_check.py` 会报 `No executable found`。

## 3. 一条命令实机验收

无相机、无外部 IO 的常用验收命令：

```bash
ros2 run horizon_arm_bringup run_acceptance_check.py \
  --real-hardware \
  --sdk-root "${HORIZON_SDK_ROOT}" \
  --arm-port "${HORIZON_ARM_PORT}" \
  --io-port "${HORIZON_IO_PORT}" \
  --no-camera \
  --no-io \
  --step-delay-sec 3.0
```

预期结果：

```text
PASS=57 WARN=0 FAIL=0 SKIP=6
```

报告默认输出到：

```text
${HORIZON_WS}/horizon_full_acceptance/
```

其中 `SKIP` 表示按硬件条件跳过，例如无相机或无 IO；`FAIL=0` 才表示当前验收通过。

## 4. 启动服务栈

教学和二次开发时先启动完整服务栈：

```bash
ros2 launch horizon_arm_bringup sdk_real.launch.py \
  sdk_root:="${HORIZON_SDK_ROOT}" \
  arm_port:="${HORIZON_ARM_PORT}" \
  arm_baudrate:=115200 \
  io_port:="${HORIZON_IO_PORT}" \
  io_baudrate:=115200
```

另开一个终端检查接口：

```bash
ros2 topic list | grep horizon_arm
ros2 service list | grep horizon_arm
ros2 action list | grep horizon_arm
```

## 5. 最小功能例子

查看机械臂状态：

```bash
ros2 topic echo /horizon_arm/status --once
```

使能机械臂：

```bash
ros2 service call /horizon_arm/enable std_srvs/srv/Trigger '{}'
```

小幅移动 J2 到 5 度：

```bash
ros2 action send_goal /horizon_arm/run_instruction horizon_arm_interfaces/action/RunInstruction \
'{instruction: "{\"command\":\"move_joints_deg\",\"joints\":[0,5,0,0,0,0],\"duration\":2.0}"}'
sleep 3
```

张开夹爪：

```bash
ros2 service call /horizon_arm/set_gripper_state horizon_arm_interfaces/srv/SetGripperState \
'{open: true, current_ma: 1200}'
sleep 3
```

视觉抓取 dry-run：

```bash
ros2 service call /horizon_arm/visual_grasp_ex horizon_arm_interfaces/srv/VisualGraspEx \
'{mode: "click", pipeline: "click", dry_run: true, use_click: true, use_depth: true, u: 320.0, v: 240.0}'
```

更多输入、输出和预期结果见 [核心功能教学开发指南](./delivery_check/horizon_arm_ws/docs/Horizon_ARM_ROS2_2.0核心功能教学开发指南.md)。

## 6. 常见问题

### `No executable found`

执行：

```bash
cd "${HORIZON_WS}"
chmod +x src/horizon_arm_bringup/scripts/run_acceptance_check.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select horizon_arm_bringup
source install/setup.bash
ros2 pkg executables horizon_arm_bringup
```

### 无相机或 `/dev/video0` warning

无相机验收时加：

```bash
--no-camera
```

报告中相机实采项目会显示 `SKIP`。

### 无外部 IO

无外部 IO 验收时加：

```bash
--no-io
```

### `rcl_shutdown already called`

这是旧版节点退出时的 shutdown 噪声。同步新版后重新编译：

```bash
colcon build --symlink-install --packages-select \
  horizon_arm_driver \
  horizon_arm_control \
  horizon_arm_bringup
source install/setup.bash
```

## 7. 安全提示

- `--real-hardware` 会产生真实机械臂动作和夹爪动作。
- 实机动作之间建议等待 3 到 5 秒。
- 教学和调试先用小角度、小距离、`dry_run`。
- 现场异常时优先调用 `/horizon_arm/emergency_stop`。
