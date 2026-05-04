# horizon_arm_bringup

`horizon_arm_bringup` 负责把驱动、控制服务、视觉服务、示教服务、Joy-Con 服务、具身智能服务和验收脚本组织成可直接启动的 ROS2 工作流。

本文命令使用 `HORIZON_WS`、`HORIZON_SDK_ROOT`、`HORIZON_ARM_PORT`、`HORIZON_IO_PORT` 等变量。变量含义和设置方法见工作空间根目录的 `docs/部署说明.md` 第 0 节。

## 1. 安装入口脚本

`run_acceptance_check.py` 是一条命令验收入口，必须有执行权限并经过 `colcon build` 安装。

```bash
cd "${HORIZON_WS}"
chmod +x src/horizon_arm_bringup/scripts/run_acceptance_check.py

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select horizon_arm_bringup
source install/setup.bash

ros2 pkg executables horizon_arm_bringup
```

预期看到：

```text
horizon_arm_bringup run_acceptance_check.py
```

## 2. 一条命令验收

逻辑模式：启动完整 ROS2 服务栈，但不连接真实硬件，不执行真实机械臂动作。报告中会标注实机动作未进行。

```bash
ros2 run horizon_arm_bringup run_acceptance_check.py \
  --sdk-root "${HORIZON_SDK_ROOT}"
```

实机模式：连接机械臂、夹爪和可用 IO，并执行验收矩阵。

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

常用参数：

| 参数 | 含义 |
|---|---|
| `--real-hardware` | 连接真实机械臂并执行真实动作 |
| `--sdk-root` | Linux SDK 根目录 |
| `--report-dir` | 报告输出目录，默认 `horizon_full_acceptance/` |
| `--arm-port` | 机械臂串口 |
| `--io-port` | IO 串口 |
| `--no-camera` | 当前无相机，跳过相机实采测试 |
| `--no-io` | 当前无外部 IO，跳过真实 IO 输出 |
| `--step-delay-sec` | 每个真实动作后的稳定等待时间，现场建议 3 到 5 秒 |
| `--dry-run` | 只打印最终 `ros2 launch` 命令，不执行 |

报告输出：

```text
horizon_full_acceptance/horizon_arm_system_check_results.json
horizon_full_acceptance/horizon_arm_system_check_results.html
```

## 3. 常用 launch

### `sdk_real.launch.py`

用途：真实硬件教学、二次开发和联调的主入口。

启动：

```bash
ros2 launch horizon_arm_bringup sdk_real.launch.py \
  sdk_root:="${HORIZON_SDK_ROOT}" \
  arm_port:="${HORIZON_ARM_PORT}" \
  arm_baudrate:=115200 \
  io_port:="${HORIZON_IO_PORT}" \
  io_baudrate:=115200
```

启动内容：

| 节点 | 说明 |
|---|---|
| `horizon_arm_driver` | 机械臂驱动 |
| `run_instruction_server` | 高层动作入口 |
| `digital_output_server` | IO 输出 |
| `visual_grasp_server` | 视觉抓取、检测、配置 |
| `follow_grasp_server` | 目标跟随 |
| `joycon_server` | Joy-Con 控制 |
| `teaching_server` | 示教点动和示教程序 |
| `embodied_server` | 具身智能 |

### `acceptance_check.launch.py`

用途：一条命令验收脚本内部调用。通常不需要手动调用。

关键参数：

| 参数 | 含义 |
|---|---|
| `real_hardware` | 是否真实连接硬件 |
| `report_dir` | 报告目录 |
| `live_step_delay_sec` | 实机动作间等待 |
| `camera_hardware_available` | 是否有相机 |
| `io_hardware_available` | 是否有 IO |

### 其他 launch

| launch | 用途 |
|---|---|
| `driver_real.launch.py` | 只启动真实硬件驱动 |
| `driver_sim.launch.py` | 启动仿真或回显模式驱动 |
| `moveit_real.launch.py` | 启动 MoveIt 与真实驱动配套环境 |
| `display_check.launch.py` | RViz 可视化检查 |
| `preset_action_real.launch.py` | 执行预设动作相关工作流 |
| `rviz_joint_state_check.launch.py` | RViz 关节状态检查 |
| `rviz_real_consistency_check.launch.py` | RViz 与真实状态一致性检查 |

## 4. 辅助脚本

| 脚本 | 作用 |
|---|---|
| `run_acceptance_check.py` | 一条命令验收入口 |
| `planning_scene_safety_objects.py` | 向规划场景注入安全物体 |
| `rviz_pose_consistency_monitor.py` | 监测 RViz 姿态一致性 |
| `execute_preset_action.py` | 执行预设动作 |
| `wait_for_joint_state_stability.py` | 等待关节状态稳定 |

## 5. 常见问题

`No executable found`：通常是脚本没有执行权限，或 `horizon_arm_bringup` 没重新编译安装。执行第 1 节命令修复。

无相机：验收命令加 `--no-camera`。

无外部 IO：验收命令加 `--no-io`。

动作太密：实机验收使用 `--step-delay-sec 3.0` 或更大值。

