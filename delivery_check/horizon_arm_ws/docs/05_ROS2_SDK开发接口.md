# Horizon Arm ROS2 2.0 开发接口说明

更新时间：2026-05-04

本文面向二次开发人员，说明 Horizon Arm ROS2 包提供的话题、服务、动作和 Python SDK。部署、编译、验收流程见 [部署说明](./部署说明.md)，按功能演示的教学流程见 [核心功能教学开发指南](./Horizon_ARM_ROS2_2.0核心功能教学开发指南.md)。

本文命令使用 `HORIZON_WS`、`HORIZON_SDK_ROOT`、`HORIZON_ARM_PORT`、`HORIZON_IO_PORT` 等变量。变量含义和设置方法见 [部署说明](./部署说明.md) 第 0 节。

## 1. 使用前提

启动 ROS2 环境：

```bash
cd "${HORIZON_WS}"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

启动真实硬件服务栈：

```bash
ros2 launch horizon_arm_bringup sdk_real.launch.py \
  sdk_root:="${HORIZON_SDK_ROOT}" \
  arm_port:="${HORIZON_ARM_PORT}" \
  arm_baudrate:=115200 \
  io_port:="${HORIZON_IO_PORT}" \
  io_baudrate:=115200
```

验证入口是否安装成功：

```bash
ros2 pkg executables horizon_arm_bringup
```

应看到：

```text
horizon_arm_bringup run_acceptance_check.py
```

如果 `ros2 run horizon_arm_bringup run_acceptance_check.py` 报 `No executable found`，执行：

```bash
chmod +x src/horizon_arm_bringup/scripts/run_acceptance_check.py
colcon build --symlink-install --packages-select horizon_arm_bringup
source install/setup.bash
```

## 2. 接口总览

| 类型 | 名称 | 用途 |
|---|---|---|
| Topic | `/horizon_arm/status` | 发布机械臂连接、使能、关节角和告警 |
| Topic | `/horizon_arm/joint_states` | 发布 6 轴关节状态，供业务程序读取 |
| Topic | `/joint_states` | 标准 JointState，供 RViz、MoveIt 使用 |
| Service | `/horizon_arm/enable` | 机械臂使能 |
| Service | `/horizon_arm/disable` | 机械臂失能 |
| Service | `/horizon_arm/emergency_stop` | 急停 |
| Service | `/horizon_arm/set_digital_output` | 外部 IO 数字输出 |
| Service | `/horizon_arm/set_gripper_state` | 夹爪开合 |
| Service | `/horizon_arm/visual_grasp` | 基础视觉抓取 |
| Service | `/horizon_arm/visual_grasp_ex` | 增强视觉抓取和 dry-run 规划 |
| Service | `/horizon_arm/vision_config` | 视觉参数设置、查询、清除 |
| Service | `/horizon_arm/pick_hsv` | 从图像像素采样 HSV |
| Service | `/horizon_arm/detect_target` | 目标检测 |
| Service | `/horizon_arm/follow_grasp_control` | 基础跟随抓取控制 |
| Service | `/horizon_arm/follow_target` | 增强目标跟随 |
| Service | `/horizon_arm/joycon_control` | Joy-Con 基础控制 |
| Service | `/horizon_arm/joycon_advanced_control` | Joy-Con 高级控制和参数配置 |
| Service | `/horizon_arm/teach_jog` | 示教点动、笛卡尔点动、dry-run 规划 |
| Service | `/horizon_arm/embodied_instruction` | 兼容版具身智能自然语言入口 |
| Service | `/horizon_arm/embodied_command` | 扩展具身智能命令入口 |
| Action | `/horizon_arm_controller/follow_joint_trajectory` | 标准关节轨迹执行 |
| Action | `/horizon_arm/run_instruction` | 高层统一动作入口 |
| Action | `/horizon_arm/teaching_program` | 示教程序保存、加载、校验、运行 |

## 3. 话题接口

### 3.1 `/horizon_arm/status`

功能：发布机械臂运行状态，适合业务程序判断硬件是否在线、是否使能、是否有告警。

类型：`horizon_arm_interfaces/msg/ArmStatus`

字段说明：

| 字段 | 类型 | 含义 |
|---|---|---|
| `stamp` | `builtin_interfaces/Time` | 状态采样时间 |
| `hardware_connected` | `bool` | 驱动是否已连接机械臂硬件 |
| `motors_enabled` | `bool` | 电机是否已使能 |
| `joint_position_deg` | `float64[]` | 6 个关节角度，单位度，顺序为 J1 到 J6 |
| `joint_velocity_deg_s` | `float64[]` | 6 个关节速度，单位度/秒 |
| `warnings` | `string[]` | 驱动层告警或诊断信息 |

示例：

```bash
ros2 topic echo /horizon_arm/status --once
```

预期结果：如果实机连接正常，应看到 `hardware_connected: true`、`motors_enabled: true`，并且 `joint_position_deg` 有 6 个数值。若 `warnings` 不为空，需要先处理告警再执行真实运动。

### 3.2 `/horizon_arm/joint_states`

功能：发布机械臂关节状态，给业务脚本或教学程序读取当前角度。

类型：`sensor_msgs/msg/JointState`

字段说明：

| 字段 | 类型 | 含义 |
|---|---|---|
| `name` | `string[]` | 关节名，通常为 `joint_1` 到 `joint_6` |
| `position` | `float64[]` | 关节角，单位弧度 |
| `velocity` | `float64[]` | 关节速度，单位弧度/秒 |
| `effort` | `float64[]` | 力矩/电流类字段，当前主要用于兼容 |

示例：

```bash
ros2 topic echo /horizon_arm/joint_states --once
```

预期结果：输出中 `name` 和 `position` 都有 6 个元素。需要角度时可用 `角度 = 弧度 * 180 / 3.1415926` 换算。

### 3.3 `/joint_states`

功能：标准 ROS2 关节状态话题，主要给 RViz、MoveIt、robot_state_publisher 使用。

类型：`sensor_msgs/msg/JointState`

示例：

```bash
ros2 topic hz /joint_states
```

预期结果：能看到稳定发布频率。默认配置下状态发布频率约为 `5 Hz`，具体值受驱动参数 `state_publish_rate_hz` 影响。

## 4. 基础控制服务

### 4.1 `/horizon_arm/enable`

功能：使能机械臂电机。

类型：`std_srvs/srv/Trigger`

输入：无字段。

输出：

| 字段 | 含义 |
|---|---|
| `success` | `true` 表示使能成功 |
| `message` | 驱动返回的说明，例如“机械臂电机已使能。” |

示例：

```bash
ros2 service call /horizon_arm/enable std_srvs/srv/Trigger '{}'
```

预期结果：`success: true`。随后查看 `/horizon_arm/status`，`motors_enabled` 应为 `true`。

### 4.2 `/horizon_arm/disable`

功能：关闭电机使能。通常用于演示结束、检修前或需要释放电机时。

类型：`std_srvs/srv/Trigger`

示例：

```bash
ros2 service call /horizon_arm/disable std_srvs/srv/Trigger '{}'
```

预期结果：`success: true`。随后 `/horizon_arm/status` 中 `motors_enabled` 会变为 `false`。

### 4.3 `/horizon_arm/emergency_stop`

功能：急停。用于异常运动、碰撞风险或需要立即中断时。

类型：`std_srvs/srv/Trigger`

示例：

```bash
ros2 service call /horizon_arm/emergency_stop std_srvs/srv/Trigger '{}'
```

预期结果：`success: true`。急停后不要马上继续发运动命令，应先确认现场安全和硬件状态。

## 5. IO 与夹爪服务

### 5.1 `/horizon_arm/set_digital_output`

功能：控制外部 IO 模块的数字输出通道。

类型：`horizon_arm_interfaces/srv/SetDigitalOutput`

输入字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `channel` | `uint8` | 输出通道号，从 0 开始 |
| `state` | `bool` | `true` 输出高电平或打开，`false` 输出低电平或关闭 |

输出字段：

| 字段 | 含义 |
|---|---|
| `success` | 是否执行成功 |
| `message` | 返回说明 |

示例：

```bash
ros2 service call /horizon_arm/set_digital_output horizon_arm_interfaces/srv/SetDigitalOutput \
'{channel: 0, state: true}'
```

预期结果：`success: true`，第 0 路 IO 输出打开。没有外部 IO 硬件时不要做真实输出测试，验收命令使用 `--no-io`。

### 5.2 `/horizon_arm/set_gripper_state`

功能：控制夹爪张开或闭合。

类型：`horizon_arm_interfaces/srv/SetGripperState`

输入字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `open` | `bool` | `true` 张开夹爪，`false` 闭合夹爪 |
| `current_ma` | `int32` | 夹爪电流，单位 mA，常用 `1200` |

输出字段：

| 字段 | 含义 |
|---|---|
| `success` | 是否执行成功 |
| `message` | 返回说明 |

闭合示例：

```bash
ros2 service call /horizon_arm/set_gripper_state horizon_arm_interfaces/srv/SetGripperState \
'{open: false, current_ma: 1200}'
```

张开示例：

```bash
ros2 service call /horizon_arm/set_gripper_state horizon_arm_interfaces/srv/SetGripperState \
'{open: true, current_ma: 1200}'
```

预期结果：返回 `success: true`，夹爪完成对应动作。连续开合之间建议 `sleep 3`，给夹爪和机械结构留出稳定时间。

## 6. 运动动作接口

### 6.1 `/horizon_arm_controller/follow_joint_trajectory`

功能：标准 ROS2 关节轨迹 Action。MoveIt、Python SDK 和高层指令最终都会通过它执行关节运动。

类型：`control_msgs/action/FollowJointTrajectory`

关键输入字段：

| 字段 | 含义 |
|---|---|
| `trajectory.joint_names` | 关节名，推荐 `joint_1` 到 `joint_6` |
| `trajectory.points[].positions` | 每个轨迹点的目标关节角，单位弧度 |
| `trajectory.points[].time_from_start` | 从轨迹开始到该点的时间 |

关键输出字段：

| 字段 | 含义 |
|---|---|
| `error_code` | `0` 代表成功，非 0 代表失败 |
| `error_string` | 失败说明 |

示例：移动 J2 到 5 度，其余为 0 度。这里位置单位是弧度，5 度约等于 `0.0872665`。

```bash
ros2 action send_goal /horizon_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
'{trajectory: {joint_names: ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"], points: [{positions: [0.0,0.0872665,0.0,0.0,0.0,0.0], time_from_start: {sec: 2, nanosec: 0}}]}}'
```

预期结果：Action 返回 `error_code: 0`，机械臂在约 2 秒内完成小幅运动。实机连续运动之间建议再等待 3 秒。

### 6.2 `/horizon_arm/run_instruction`

功能：统一高层动作入口。适合教学、脚本、验收程序调用，内部会解析字符串或 JSON，再调用基础服务或轨迹 Action。

类型：`horizon_arm_interfaces/action/RunInstruction`

Goal 字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `instruction` | `string` | 指令文本。可以是短文本，也可以是 JSON 字符串 |

Result 字段：

| 字段 | 含义 |
|---|---|
| `success` | 指令是否执行成功 |
| `message` | 执行说明 |

Feedback 字段：

| 字段 | 含义 |
|---|---|
| `stage` | 当前阶段，如 `parse`、`motion`、`execute`、`done` |
| `progress` | 进度，0.0 到 1.0 |
| `detail` | 当前阶段说明 |

支持的短文本：

| 指令 | 含义 |
|---|---|
| `enable` | 使能 |
| `disable` | 失能 |
| `estop` 或 `emergency_stop` | 急停 |
| `preset:点头` | 执行名为“点头”的预设动作 |
| `set_do:0=1` | 第 0 路 IO 输出打开 |
| `set_do:0=0` | 第 0 路 IO 输出关闭 |
| `gripper:open` | 张开夹爪 |
| `gripper:close` | 闭合夹爪 |
| `gripper:open=1200` | 以 1200 mA 张开夹爪 |
| `gripper:close=1200` | 以 1200 mA 闭合夹爪 |

支持的 JSON 指令：

```json
{"command":"enable"}
{"command":"disable"}
{"command":"emergency_stop"}
{"command":"preset","name":"点头"}
{"command":"move_joints_deg","joints":[0,5,0,0,0,0],"duration":2.0}
{"command":"move_joints_rad","joints":[0.0,0.1,0.0,0.0,0.0,0.0],"duration":2.0}
{"command":"set_digital_output","channel":0,"state":true}
{"command":"set_gripper_state","open":true,"current_ma":1200}
```

示例 1：使能。

```bash
ros2 action send_goal /horizon_arm/run_instruction horizon_arm_interfaces/action/RunInstruction \
'{instruction: "enable"}'
```

预期结果：`success: true`，返回消息提示电机已使能。

示例 2：小幅关节运动。

```bash
ros2 action send_goal /horizon_arm/run_instruction horizon_arm_interfaces/action/RunInstruction \
'{instruction: "{\"command\":\"move_joints_deg\",\"joints\":[0,5,0,0,0,0],\"duration\":2.0}"}'
```

输入含义：`joints` 是 6 个关节目标角，单位度；`duration` 是运动时间，单位秒。

预期结果：J2 做 5 度小幅运动，Action 返回 `move_joints_deg completed successfully`。执行后建议：

```bash
sleep 3
```

示例 3：预设动作。

```bash
ros2 action send_goal /horizon_arm/run_instruction horizon_arm_interfaces/action/RunInstruction \
'{instruction: "preset:点头"}'
```

预期结果：执行 SDK 预设文件中的“点头”动作。预设文件默认来自：

```text
${HORIZON_SDK_ROOT}/config/embodied_config/preset_actions.json
```

## 7. 视觉服务

### 7.1 `/horizon_arm/vision_config`

功能：设置、查询或清除视觉检测参数，供 HSV 检测、YOLO 检测、视觉抓取和跟随使用。

类型：`horizon_arm_interfaces/srv/VisionConfig`

输入字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `command` | `string` | `set`/`update` 设置，`get`/`status` 查询，`clear` 清除 |
| `pipeline` | `string` | 检测管线，如 `hsv`、`yolo`、`click` |
| `target_class` | `string` | 目标类别名，如 `red_block`、`person` |
| `conf_thres` | `float32` | 置信度阈值，常用 0.35 到 0.7 |
| `iou_thres` | `float32` | YOLO NMS IoU 阈值，常用 0.45 |
| `interval_sec` | `float32` | 检测或跟随循环间隔 |
| `hsv_h_min/hsv_h_max` | `int32` | HSV 色相范围 |
| `hsv_s_min/hsv_s_max` | `int32` | HSV 饱和度范围 |
| `hsv_v_min/hsv_v_max` | `int32` | HSV 明度范围 |
| `depth_min_m/depth_max_m` | `float32` | 深度过滤范围，单位米 |
| `pixel_to_mm_scale` | `float32` | 像素到毫米比例，标定后使用 |
| `model_path` | `string` | YOLO 模型路径，可为空 |
| `camera_name` | `string` | 相机名，可为空 |
| `options_json` | `string` | 扩展参数 JSON 字符串 |

输出字段：

| 字段 | 含义 |
|---|---|
| `success` | 是否处理成功 |
| `message` | 说明 |
| `config_json` | 当前配置 JSON |

示例：

```bash
ros2 service call /horizon_arm/vision_config horizon_arm_interfaces/srv/VisionConfig \
'{command: "set", pipeline: "hsv", target_class: "red_block", conf_thres: 0.5, iou_thres: 0.45, interval_sec: 0.2, hsv_h_min: 0, hsv_h_max: 12, hsv_s_min: 80, hsv_s_max: 255, hsv_v_min: 60, hsv_v_max: 255, depth_min_m: 0.08, depth_max_m: 0.8}'
```

预期结果：`success: true`，`config_json` 中能看到 `pipeline: hsv` 和对应 HSV 参数。

### 7.2 `/horizon_arm/pick_hsv`

功能：从相机图像某个像素点附近采样 HSV 值，帮助调 HSV 阈值。

类型：`horizon_arm_interfaces/srv/PickHSV`

输入字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `u` | `float32` | 图像横坐标，像素 |
| `v` | `float32` | 图像纵坐标，像素 |
| `window_size` | `int32` | 采样窗口大小，常用 `9` |
| `use_depth_filter` | `bool` | 是否按深度过滤 |
| `depth_min_m/depth_max_m` | `float32` | 深度有效范围，单位米 |

输出字段：

| 字段 | 含义 |
|---|---|
| `h/s/v` | 采样中心 HSV 值 |
| `h_min/h_max/s_min/s_max/v_min/v_max` | 推荐阈值范围 |
| `depth_m` | 采样点深度，单位米 |
| `message` | 说明 |

示例：

```bash
ros2 service call /horizon_arm/pick_hsv horizon_arm_interfaces/srv/PickHSV \
'{u: 320.0, v: 240.0, window_size: 9, use_depth_filter: true, depth_min_m: 0.08, depth_max_m: 0.8}'
```

预期结果：有相机时返回采样 HSV 和推荐范围；无相机时会失败或提示无法打开相机。无相机场景验收使用 `--no-camera`。

### 7.3 `/horizon_arm/detect_target`

功能：执行目标检测，返回目标框、中心点、类别、分数和深度。

类型：`horizon_arm_interfaces/srv/DetectTarget`

输入字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `pipeline` | `string` | `hsv` 或 `yolo` |
| `target_class` | `string` | 目标类别，可为空 |
| `conf_thres` | `float32` | 置信度阈值 |
| `use_hsv` | `bool` | 是否使用 HSV 参数 |
| `use_depth` | `bool` | 是否读取深度 |
| `depth_min_m/depth_max_m` | `float32` | 深度过滤范围 |

输出字段：

| 字段 | 含义 |
|---|---|
| `success` | 检测是否成功 |
| `count` | 目标数量 |
| `bboxes` | 目标框数组，按 `[x1,y1,x2,y2,...]` 展平 |
| `centers` | 中心点数组，按 `[u,v,...]` 展平 |
| `scores` | 每个目标的置信度 |
| `class_names` | 类别名 |
| `depths_m` | 每个目标中心深度，单位米 |
| `message` | 说明 |

示例：

```bash
ros2 service call /horizon_arm/detect_target horizon_arm_interfaces/srv/DetectTarget \
'{pipeline: "hsv", target_class: "red_block", conf_thres: 0.5, use_hsv: true, use_depth: true, depth_min_m: 0.08, depth_max_m: 0.8}'
```

预期结果：检测到目标时 `count > 0`，`bboxes` 每 4 个数代表一个框，`centers` 每 2 个数代表一个中心点。

### 7.4 `/horizon_arm/visual_grasp`

功能：基础视觉抓取接口。支持点击点或 bbox，适合兼容旧流程。

类型：`horizon_arm_interfaces/srv/VisualGrasp`

输入字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `dry_run` | `bool` | `true` 只检查链路或规划，不执行真实抓取 |
| `use_bbox` | `bool` | 是否使用目标框 |
| `u/v` | `float32` | 点击点像素坐标 |
| `x1/y1/x2/y2` | `float32` | 目标框左上和右下像素坐标 |

输出字段：

| 字段 | 含义 |
|---|---|
| `success` | 是否成功 |
| `message` | 说明 |

示例：

```bash
ros2 service call /horizon_arm/visual_grasp horizon_arm_interfaces/srv/VisualGrasp \
'{dry_run: true, use_bbox: false, u: 320.0, v: 240.0}'
```

预期结果：返回 `success: true` 表示视觉 wrapper 链路可用；真实抓取前应先完成相机标定和目标检测验证。

开发顺序建议：

1. 先用 `vision_config` 固定目标类别和阈值。
2. 再用 `detect_target` 确认目标可见。
3. 用 `visual_grasp_ex` 做 `dry_run`，检查三维目标和姿态。
4. 最后才做真实抓取。

### 7.5 `/horizon_arm/visual_grasp_ex`

功能：增强视觉抓取接口。支持点击、bbox、HSV、深度估计和 dry-run，是推荐使用的视觉抓取入口。

类型：`horizon_arm_interfaces/srv/VisualGraspEx`

输入字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `mode` | `string` | 抓取模式，如 `click`、`bbox`、`hsv` |
| `pipeline` | `string` | 检测管线，如 `click`、`hsv`、`yolo` |
| `target_class` | `string` | 目标类别 |
| `dry_run` | `bool` | `true` 只返回规划结果，不执行真实抓取 |
| `use_click` | `bool` | 使用 `u/v` 点击点 |
| `use_bbox` | `bool` | 使用 `x1/y1/x2/y2` 目标框 |
| `use_hsv` | `bool` | 使用 HSV 检测 |
| `use_depth` | `bool` | 使用深度估计 |
| `u/v` | `float32` | 点击点像素坐标 |
| `x1/y1/x2/y2` | `float32` | bbox 坐标 |
| `z_offset_m` | `float32` | 抓取高度补偿，单位米 |
| `approach_height_m` | `float32` | 预抓取上方高度，单位米 |
| `grasp_depth_m` | `float32` | 下探深度，单位米 |
| `pre_grasp_rpy` | `float32[]` | 预抓取姿态 RPY，单位弧度 |
| `options_json` | `string` | 扩展参数 JSON |

输出字段：

| 字段 | 含义 |
|---|---|
| `success` | 是否成功 |
| `message` | 说明 |
| `target_xyz` | 估计的目标三维坐标 |
| `target_rpy` | 估计的目标姿态 |
| `result_json` | 完整请求与规划结果 |

dry-run 示例：

```bash
ros2 service call /horizon_arm/visual_grasp_ex horizon_arm_interfaces/srv/VisualGraspEx \
'{mode: "click", pipeline: "click", dry_run: true, use_click: true, use_depth: true, u: 320.0, v: 240.0, approach_height_m: 0.08, grasp_depth_m: 0.02}'
```

预期结果：`success: true`，`message` 类似 `visual grasp ex dry-run accepted: click`，不会移动机械臂。

开发者常见做法是把 `detect_target` 的第一组 `bboxes` 直接传给 `visual_grasp_ex`。这样才能把“检测”和“抓取”串成完整流程，而不是只会调一个独立接口。

## 8. 跟随服务

### 8.1 `/horizon_arm/follow_grasp_control`

功能：兼容版跟随抓取控制，提供 `status/start/stop/health`。

类型：`horizon_arm_interfaces/srv/FollowGraspControl`

输入字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `command` | `string` | `status`、`health`、`start`、`stop` |
| `target_class` | `string` | 跟随目标类别 |
| `conf_thres` | `float32` | 检测置信度阈值 |
| `interval_sec` | `float32` | 跟随循环间隔 |

输出字段：

| 字段 | 含义 |
|---|---|
| `success` | 是否成功 |
| `running` | 跟随是否正在运行 |
| `message` | 说明 |

示例：

```bash
ros2 service call /horizon_arm/follow_grasp_control horizon_arm_interfaces/srv/FollowGraspControl \
'{command: "status"}'
```

预期结果：返回当前是否正在跟随，通常未启动时 `running: false`。

开发建议：

1. 先查询 `status`。
2. 再用 `set_target` 或 `start`。
3. 需要人工接管时用 `pause`。
4. 结束时用 `stop`。

### 8.2 `/horizon_arm/follow_target`

功能：增强目标跟随接口。可用 YOLO、HSV 或手动目标框，支持暂停、恢复和状态查询。

类型：`horizon_arm_interfaces/srv/FollowTarget`

输入字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `command` | `string` | `status`、`health`、`start`、`resume`、`stop`、`pause`、`set_target` |
| `mode` | `string` | 跟随模式，如 `manual`、`yolo`、`hsv` |
| `pipeline` | `string` | 检测管线 |
| `target_class` | `string` | 目标类别 |
| `conf_thres` | `float32` | 置信度阈值 |
| `interval_sec` | `float32` | 跟随循环间隔 |
| `follow_distance_m` | `float32` | 期望跟随距离 |
| `deadband_px` | `float32` | 图像中心死区，像素 |
| `max_linear_speed` | `float32` | 最大线速度，0 表示由 SDK 默认控制 |
| `max_angular_speed` | `float32` | 最大角速度，0 表示由 SDK 默认控制 |
| `use_depth` | `bool` | 是否使用深度 |
| `auto_grasp` | `bool` | 是否到达后自动抓取 |
| `options_json` | `string` | 扩展参数，例如手动框或 HSV 范围 |

输出字段：

| 字段 | 含义 |
|---|---|
| `success` | 是否成功 |
| `running` | 跟随是否运行中 |
| `message` | 说明 |
| `state_json` | 当前跟随状态、请求参数、最近目标中心 |

状态查询示例：

```bash
ros2 service call /horizon_arm/follow_target horizon_arm_interfaces/srv/FollowTarget \
'{command: "status"}'
```

手动目标框示例：

```bash
ros2 service call /horizon_arm/follow_target horizon_arm_interfaces/srv/FollowTarget \
'{command: "set_target", mode: "manual", pipeline: "manual", options_json: "{\"x1\":240,\"y1\":160,\"x2\":360,\"y2\":280}"}'
```

预期结果：`state_json` 中 `manual_target_ready` 变为 `true`。有相机和安全空间后再执行 `start`。

### 8.3 跟随开发链路

典型链路是：

```text
vision_config -> detect_target/set_target -> follow_target(start) -> follow_target(status) -> follow_target(pause/stop)
```

重点字段：

| 字段 | 作用 |
|---|---|
| `follow_distance_m` | 目标跟随距离 |
| `deadband_px` | 图像中心死区，减小抖动 |
| `max_linear_speed` | 线速度上限 |
| `max_angular_speed` | 角速度上限 |
| `auto_grasp` | 到位后是否自动抓取 |

`state_json` 要展开看，不要只看 `success`。

## 9. Joy-Con 服务

### 9.1 `/horizon_arm/joycon_control`

功能：Joy-Con 基础控制。

类型：`horizon_arm_interfaces/srv/JoyconControl`

输入字段：

| 字段 | 含义 |
|---|---|
| `command` | `status`、`connect`、`disconnect`、`start`、`stop` |

输出字段：

| 字段 | 含义 |
|---|---|
| `success` | 是否成功 |
| `running` | 控制是否运行 |
| `message` | 说明 |

示例：

```bash
ros2 service call /horizon_arm/joycon_control horizon_arm_interfaces/srv/JoyconControl \
'{command: "status"}'
```

预期结果：未连接 Joy-Con 时也应能返回 wrapper 状态；真实手柄控制需先 `connect`，再 `start`。

这个接口适合做状态和连通性检查，但不足以支撑完整控制面板，所以真实开发通常会转到高级接口。

### 9.2 `/horizon_arm/joycon_advanced_control`

功能：Joy-Con 高级控制，支持模式切换、速度配置、姿态模式、工作空间限制等。

类型：`horizon_arm_interfaces/srv/JoyconAdvancedControl`

常用命令：

| `command` | 含义 |
|---|---|
| `status` | 查询状态 |
| `connect` / `disconnect` | 连接或断开 Joy-Con |
| `start` / `stop` | 开始或停止控制 |
| `pause` / `resume` | 暂停或恢复 |
| `set_mode` | 设置 `mode` 为 `cartesian` 或 `joint` |
| `enable_attitude` / `disable_attitude` | 启用或关闭姿态控制 |
| `set_attitude_mode` | 设置姿态模式 |
| `set_dual_attitude` | 设置双手柄姿态控制 |
| `set_preferred_side` | 设置优先手柄 `left/right/auto` |
| `home` | 回到 SDK home 位 |
| `hardware_zero` | 回硬件零点 |
| `emergency_stop` | 急停 |
| `configure_speed` | 配置速度档位 |
| `configure_basic` | 配置摇杆死区等基础参数 |
| `configure_cartesian` | 配置笛卡尔控制步长和速度 |
| `configure_joint` | 配置关节控制步长和速度 |
| `configure_workspace` | 配置工作空间半径和高度限制 |
| `input_status` | 查询手柄输入状态 |

常用输入字段分组：

| 字段 | 含义 |
|---|---|
| `mode` | 控制模式，常用 `cartesian`、`joint` |
| `attitude_mode` | 姿态模式名或编号 |
| `enabled` | 配置开关，部分命令使用 |
| `dual_arm` | 是否启用双手柄/双臂相关模式 |
| `preferred_side` | `left`、`right`、`auto` |
| `speed_index` | 当前速度档位 |
| `speed_levels` | 速度档位数组 |
| `stick_deadzone` | 摇杆死区 |
| `cartesian_position_step` | 笛卡尔平移步长 |
| `cartesian_rotation_step` | 笛卡尔旋转步长 |
| `cartesian_max_speed` | 笛卡尔最大线速度 |
| `cartesian_max_angular_speed` | 笛卡尔最大角速度 |
| `joint_angle_step` | 关节点动角度步长 |
| `joint_max_speed` | 关节最大速度 |
| `joint_acceleration/joint_deceleration` | 关节加减速度 |
| `workspace_min_radius/workspace_max_radius` | 工作空间半径限制 |
| `workspace_min_z/workspace_max_z` | 工作空间 Z 方向限制 |

输出字段：

| 字段 | 含义 |
|---|---|
| `success` | 是否成功 |
| `running` | 是否运行中 |
| `mode` | 当前控制模式 |
| `attitude_mode` | 当前姿态模式 |
| `message` | 说明 |
| `status_json` | SDK 状态 JSON |
| `input_json` | 手柄输入状态 JSON |

示例：切换到关节模式。

```bash
ros2 service call /horizon_arm/joycon_advanced_control horizon_arm_interfaces/srv/JoyconAdvancedControl \
'{command: "set_mode", mode: "joint"}'
```

预期结果：`success: true`，`mode` 返回 `joint` 或状态 JSON 中显示关节模式。

### 9.3 Joy-Con 开发顺序

建议顺序如下：

1. `status` 看连接状态。
2. `connect` 连接设备。
3. `configure_basic` 设置死区。
4. `set_mode` 切换控制模式。
5. `configure_joint` 或 `configure_cartesian` 调整手感。
6. `configure_workspace` 限定安全范围。
7. `start` 开始控制。
8. 结束时 `stop`，必要时 `disconnect`。

常见字段含义：

| 字段 | 含义 |
|---|---|
| `speed_levels` | 速度档位列表 |
| `speed_index` | 当前档位 |
| `stick_deadzone` | 摇杆死区 |
| `cartesian_position_step` | 笛卡尔平移步长 |
| `cartesian_rotation_step` | 笛卡尔旋转步长 |
| `joint_angle_step` | 关节点动步长 |
| `workspace_min_radius/max_radius` | 活动半径限制 |
| `workspace_min_z/max_z` | Z 方向活动范围 |

## 10. 示教服务与动作

### 10.1 `/horizon_arm/teach_jog`

功能：示教点动和 dry-run 规划。可用于关节点动、关节绝对移动、基坐标平移、工具坐标平移、基坐标旋转、工具坐标旋转、笛卡尔目标位姿规划。

类型：`horizon_arm_interfaces/srv/TeachJog`

输入字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `command` | `string` | `status`、`joint_jog`、`joint_move`、`base_translate`、`tool_translate`、`base_rotate`、`tool_rotate`、`cartesian_move`、`set_interpolation`、`stop` |
| `frame` | `string` | 坐标系，如 `base`、`tool` |
| `axis` | `string` | 轴向，如 `x`、`y`、`z`、`rx`、`ry`、`rz` |
| `joint_index` | `int32` | 关节编号，当前按 1 到 6 使用 |
| `delta` | `float32` | 增量。关节单位度，平移单位 mm，旋转单位度 |
| `joint_angles` | `float32[]` | 6 轴目标角，单位度 |
| `position` | `float32[]` | 笛卡尔目标位置 `[x,y,z]`，单位 mm |
| `orientation` | `float32[]` | 笛卡尔目标姿态 `[roll,pitch,yaw]`，单位度 |
| `interpolation_type` | `string` | `joint` 或 `cartesian` |
| `max_speed` | `float32` | 关节运动最大速度 |
| `acceleration/deceleration` | `float32` | 关节加减速度 |
| `linear_velocity/angular_velocity` | `float32` | 笛卡尔线速度、角速度 |
| `linear_acceleration/angular_acceleration` | `float32` | 笛卡尔加速度 |
| `joint_max_velocities` | `float32[]` | 每个关节最大速度 |
| `joint_max_accelerations` | `float32[]` | 每个关节最大加速度 |
| `dry_run` | `bool` | `true` 只规划并返回目标角，不执行真实运动 |

输出字段：

| 字段 | 含义 |
|---|---|
| `success` | 是否成功 |
| `message` | 说明 |
| `target_joint_angles` | 规划出的 6 轴目标角，单位度 |
| `detail_json` | 当前位姿、目标位姿、插补参数等详细信息 |

示例：J2 正向点动 5 度，只规划不执行。

```bash
ros2 service call /horizon_arm/teach_jog horizon_arm_interfaces/srv/TeachJog \
'{command: "joint_jog", joint_index: 2, delta: 5.0, interpolation_type: "joint", dry_run: true}'
```

预期结果：`success: true`，`target_joint_angles` 返回规划后的 6 轴目标角；机械臂不会运动。

示例：基坐标 X 正方向平移 10 mm，只规划不执行。

```bash
ros2 service call /horizon_arm/teach_jog horizon_arm_interfaces/srv/TeachJog \
'{command: "base_translate", frame: "base", axis: "x", delta: 10.0, interpolation_type: "cartesian", linear_velocity: 150.0, angular_velocity: 90.0, dry_run: true}'
```

预期结果：`detail_json` 中包含 `current_pose`、`target_pose` 和 `target_joint_angles`。

### 10.3 示教开发链路

推荐顺序：

1. 对单个动作做 `dry_run`。
2. 保存 `target_joint_angles`。
3. 用 `teaching_program validate` 检查程序。
4. 最后 `teaching_program run` 执行。

这样开发出来的教学功能，才是“点动 -> 存点 -> 校验 -> 运行”的完整链路。

### 10.2 `/horizon_arm/teaching_program`

功能：示教程序保存、加载、校验和运行。

类型：`horizon_arm_interfaces/action/TeachingProgram`

Goal 字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `command` | `string` | `status`、`save`、`load`、`validate`、`run` |
| `program_name` | `string` | 程序名 |
| `program_path` | `string` | 程序 JSON 文件路径 |
| `program_json` | `string` | 直接传入的程序 JSON |
| `use_saved_params` | `bool` | 是否使用程序中保存的插补参数 |
| `dry_run` | `bool` | 是否只校验不执行 |

Result 字段：

| 字段 | 含义 |
|---|---|
| `success` | 是否成功 |
| `message` | 说明 |
| `result_json` | 程序点位、保存路径、校验结果等 |

Feedback 字段：

| 字段 | 含义 |
|---|---|
| `current_index` | 当前执行点序号 |
| `total` | 总点数 |
| `progress` | 进度 |
| `stage` | 阶段 |
| `detail` | 当前点说明 |

校验示例：

```bash
ros2 action send_goal /horizon_arm/teaching_program horizon_arm_interfaces/action/TeachingProgram \
'{command: "validate", dry_run: true, program_json: "{\"name\":\"demo\",\"points\":[{\"joint_angles\":[0,0,0,0,0,0]}]}"}'
```

预期结果：`success: true`，`message` 类似 `program validated: 1 joint points`，不会执行运动。

## 11. 具身智能服务

具身智能服务封装交付 SDK 的 `EmbodiedSDK`。它不是只做 health check，而是提供自然语言任务执行、函数能力查询、动作查询、历史管理和具身层急停控制。

配置文件：

```text
${HORIZON_SDK_ROOT}/config/aisdk_config.yaml
```

常用配置字段：

| 字段 | 含义 |
|---|---|
| `providers.alibaba.api_key` | 阿里云通义千问 API key |
| `providers.deepseek.api_key` | DeepSeek API key |
| `providers.*.default_params.temperature` | 模型输出随机性 |
| `providers.*.default_params.max_tokens` | 最大输出 token |
| `request.timeout` | AI 请求超时 |
| `embodied_intelligence.prompt_language` | 具身提示词语言 |

能力分层：

| 能力 | 是否需要 AI key | 是否产生真实动作 |
|---|---|---|
| `health/functions/actions/history` | 不需要 | 不产生动作 |
| `run/stream` 文本任务 | 需要 | 取决于自然语言内容 |
| `emergency_stop/set_emergency_stop/clear_emergency_stop` | 不需要 | 控制具身层急停标志 |

### 11.1 `/horizon_arm/embodied_instruction`

功能：兼容旧接口的自然语言入口，也可用于健康检查。

类型：`horizon_arm_interfaces/srv/EmbodiedInstruction`

输入字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `command` | `string` | `health` 或自然语言任务 |
| `stream` | `bool` | 是否启用流式执行 |

输出字段：

| 字段 | 含义 |
|---|---|
| `success` | 是否成功 |
| `message` | 说明 |
| `result_json` | SDK 返回结果 |

示例：

```bash
ros2 service call /horizon_arm/embodied_instruction horizon_arm_interfaces/srv/EmbodiedInstruction \
'{command: "health", stream: false}'
```

预期结果：返回 `success: true`，说明具身 wrapper 可用。

开发上它更适合做老项目兼容，不建议新系统把它当主入口。

自然语言示例：

```bash
ros2 service call /horizon_arm/embodied_instruction horizon_arm_interfaces/srv/EmbodiedInstruction \
'{command: "让机械臂执行点头动作", stream: false}'
```

预期结果：如果 AI provider 和真实硬件配置可用，SDK 会理解自然语言并调用对应动作；`result_json` 返回执行过程和结果。该兼容接口字段较少，二次开发推荐使用 `/horizon_arm/embodied_command`。

### 11.2 `/horizon_arm/embodied_command`

功能：扩展具身智能命令入口。可查询函数、动作、历史，或执行自然语言任务。

类型：`horizon_arm_interfaces/srv/EmbodiedCommand`

输入字段：

| 字段 | 类型 | 含义 |
|---|---|---|
| `command` | `string` | `health`、`functions`、`actions`、`history`、`clear_history`、`run`、`stream`、`emergency_stop`、`set_emergency_stop`、`clear_emergency_stop` |
| `instruction` | `string` | 自然语言任务文本 |
| `stream` | `bool` | 是否流式执行 |
| `provider` | `string` | AI provider，可为空使用 SDK 默认 |
| `model` | `string` | 模型名，可为空使用 SDK 默认 |
| `control_mode` | `string` | 控制模式，可为空使用 SDK 默认 |
| `options_json` | `string` | 扩展配置 JSON |

输出字段：

| 字段 | 含义 |
|---|---|
| `success` | 是否成功 |
| `message` | 说明 |
| `result_json` | 函数列表、动作列表、历史或执行结果 |

命令说明：

| `command` | 必填字段 | 作用 | 典型返回 |
|---|---|---|---|
| `health` | 无 | 查询函数列表并确认 wrapper 可用 | `result_json` 为函数说明 |
| `functions` | 无 | 查询具身函数能力 | `c_a_j/c_c_g/e_p_a/...` |
| `actions` | 无 | 查询动作列表 | 可用动作 JSON |
| `history` | 无 | 查询历史记录 | 历史数组 |
| `clear_history` | 无 | 清空历史 | `{}` |
| `run` | `instruction` | 同步执行自然语言任务 | SDK 执行结果 |
| `stream` | `instruction` | 流式执行并把事件收集为 JSON | progress/completion 事件 |
| `emergency_stop` | 无 | 触发具身层急停 | `{}` |
| `set_emergency_stop` | 无 | 设置具身层急停标志 | `{}` |
| `clear_emergency_stop` | 无 | 清除具身层急停标志 | `{}` |

函数能力含义：

| 函数 | 用途 |
|---|---|
| `c_a_j` | 控制机械臂 6 轴关节角运动 |
| `c_c_g` | 控制夹爪张开或闭合 |
| `e_p_a` | 执行预设动作 |
| `t_s_a` | 文本对话与语音播报 |
| `v_r_o` | 视觉识别物体并移动到目标附近 |
| `v_s_a` | 视觉分析并语音播报 |

查询函数示例：

```bash
ros2 service call /horizon_arm/embodied_command horizon_arm_interfaces/srv/EmbodiedCommand \
'{command: "functions"}'
```

预期结果：`result_json` 中包含 `c_a_j`、`c_c_g`、`e_p_a` 等可调用能力说明。

这些结果应该用于 UI 展示和权限控制，而不是让业务代码直接拼机械臂底层动作。

查询动作示例：

```bash
ros2 service call /horizon_arm/embodied_command horizon_arm_interfaces/srv/EmbodiedCommand \
'{command: "actions"}'
```

预期结果：返回可用动作或函数名列表。

执行自然语言任务：

```bash
ros2 service call /horizon_arm/embodied_command horizon_arm_interfaces/srv/EmbodiedCommand \
'{command: "run", instruction: "让机械臂执行点头动作", provider: "alibaba", model: "qwen-turbo", control_mode: "real_only"}'
```

输入解释：

| 字段 | 说明 |
|---|---|
| `instruction` | 业务自然语言，建议明确、短句、动作边界清楚 |
| `provider` | `alibaba` 或 `deepseek`，必须与 `aisdk_config.yaml` 中 enabled provider 对应 |
| `model` | 模型名，如 `qwen-turbo` |
| `control_mode` | `real_only` 表示只控制真实机械臂；SDK 支持的其他模式可按现场配置使用 |
| `options_json` | 可传 `{"config_path":"..."}` 指定 AI 配置文件 |

预期结果：`success: true` 时，`result_json` 包含 SDK 返回的规划和执行结果。真实动作后建议 `sleep 3`。

流式任务：

```bash
ros2 service call /horizon_arm/embodied_command horizon_arm_interfaces/srv/EmbodiedCommand \
'{command: "stream", instruction: "先张开夹爪，然后执行点头动作", stream: true, provider: "alibaba", model: "qwen-turbo", control_mode: "real_only"}'
```

预期结果：`result_json` 为事件数组，包含 `progress` 和 `completion`。当前接口是 service，会在一次响应里返回收集到的事件；如需 UI 实时进度，可基于该 wrapper 扩展 action/topic。

### 11.3 历史管理

历史在开发里通常有三个用途：

| 用途 | 说明 |
|---|---|
| 上下文追踪 | 看上一条任务是什么 |
| 调试 | 对比模型理解和执行结果 |
| 清场 | 新任务开始前清空历史 |

如果你做控制台，建议把历史做成独立面板，并提供 `clear_history` 按钮。

### 11.4 直接 SDK 调用

如果你不想经过 ROS2，也可以直接调用 SDK。这样适合写后端服务或独立脚本。

```python
import os
from Embodied_SDK.embodied import EmbodiedSDK

sdk = EmbodiedSDK(
    provider="alibaba",
    model="qwen-turbo",
    control_mode="real_only",
    config_path=os.path.join(os.environ["HORIZON_SDK_ROOT"], "config", "aisdk_config.yaml"),
)

print(sdk.get_available_functions())
print(sdk.get_available_actions())
print(sdk.run_nl_instruction("让机械臂执行点头动作"))
```

输出含义：

| 调用 | 结果 |
|---|---|
| `get_available_functions()` | 适合做能力列表页面 |
| `get_available_actions()` | 适合做动作面板 |
| `run_nl_instruction()` | 适合做实际任务执行 |

如果需要流式进度，可以改用 `run_nl_instruction_stream()`，并传入 `progress_handler` 和 `completion_handler`。

### 11.5 具身智能开发顺序

推荐顺序：

1. `functions` 看能力。
2. `actions` 看动作。
3. `history` 看上下文。
4. `run` 做小动作。
5. `stream` 做带进度的 UI。
6. `emergency_stop` 和 `set_emergency_stop` 做风险控制。

首个推荐例子是：

```text
让机械臂执行点头动作
```

这个例子足够简单，能把自然语言、SDK、机械臂和报告打通。

历史管理：

```bash
ros2 service call /horizon_arm/embodied_command horizon_arm_interfaces/srv/EmbodiedCommand \
'{command: "history"}'

ros2 service call /horizon_arm/embodied_command horizon_arm_interfaces/srv/EmbodiedCommand \
'{command: "clear_history"}'
```

具身层急停：

```bash
ros2 service call /horizon_arm/embodied_command horizon_arm_interfaces/srv/EmbodiedCommand \
'{command: "emergency_stop"}'

ros2 service call /horizon_arm/embodied_command horizon_arm_interfaces/srv/EmbodiedCommand \
'{command: "clear_emergency_stop"}'
```

注意：真实硬件异常时，还应调用底层急停：

```bash
ros2 service call /horizon_arm/emergency_stop std_srvs/srv/Trigger '{}'
```

## 12. Python SDK

Python SDK 类名：`horizon_arm_control.HorizonArmRosSdk`

用途：把上面的话题、服务、动作封装成阻塞式 Python 方法，适合教学脚本、验收脚本和二次开发。

最小示例：

```python
import time
import rclpy
from rclpy.node import Node
from horizon_arm_control import HorizonArmRosSdk

rclpy.init()
node = Node("horizon_arm_demo")
arm = HorizonArmRosSdk(node)

try:
    ready = arm.wait_until_ready(
        include_instruction=True,
        include_gripper=True,
        include_extended_wrappers=True,
    )
    print("ready:", ready)

    print(arm.enable())
    print(arm.move_joints_deg([0, 5, 0, 0, 0, 0], duration_sec=2.0))
    time.sleep(3)
    print(arm.open_gripper(current_ma=1200))
finally:
    node.destroy_node()
    rclpy.shutdown()
```

常用方法说明：

| 方法 | 输入 | 输出 | 说明 |
|---|---|---|---|
| `wait_until_ready(timeout_sec, include_*)` | 超时时间和是否等待扩展服务 | `bool` | 等待 Action/Service 可用 |
| `enable()` | `timeout_sec` | `TriggerCallResult(success,message)` | 使能 |
| `disable()` | `timeout_sec` | `TriggerCallResult` | 失能 |
| `emergency_stop()` | `timeout_sec` | `TriggerCallResult` | 急停 |
| `move_joints_deg(joints_deg,duration_sec)` | 6 轴角度，单位度 | `TrajectoryExecutionResult(success,error_code,error_string)` | 关节运动 |
| `move_joints_rad(joints_rad,duration_sec)` | 6 轴角度，单位弧度 | `TrajectoryExecutionResult` | 关节运动 |
| `execute_preset(preset_name)` | 预设名 | `TrajectoryExecutionResult` | 执行预设动作 |
| `run_instruction(instruction)` | 高层指令字符串 | `InstructionExecutionResult(success,message)` | 调 `/horizon_arm/run_instruction` |
| `set_digital_output(channel,state)` | 通道号、状态 | `DigitalOutputCallResult` | IO 输出 |
| `set_gripper_state(open,current_ma)` | 开合、电流 | `GripperCallResult` | 夹爪控制 |
| `open_gripper(current_ma)` | 电流 | `GripperCallResult` | 张开夹爪 |
| `close_gripper(current_ma)` | 电流 | `GripperCallResult` | 闭合夹爪 |
| `visual_grasp(u,v,bbox,dry_run)` | 点或框 | `SimpleServiceCallResult` | 基础视觉抓取 |
| `visual_grasp_health()` | 无 | `SimpleServiceCallResult` | 视觉 wrapper 健康检查 |
| `visual_grasp_ex(...)` | 模式、点、框、深度参数 | `JsonServiceCallResult(success,message,payload_json)` | 增强视觉抓取 |
| `configure_vision(...)` | pipeline、HSV、深度等 | `JsonServiceCallResult` | 配置视觉 |
| `pick_hsv(u,v,window_size,...)` | 像素点和窗口 | `JsonServiceCallResult` | HSV 采样 |
| `detect_target(...)` | pipeline、类别、阈值 | `JsonServiceCallResult` | 目标检测 |
| `follow_grasp_control(command,...)` | `status/start/stop` 等 | `JsonServiceCallResult` | 基础跟随 |
| `follow_target(command,...)` | 模式、距离、速度等 | `JsonServiceCallResult` | 增强跟随 |
| `joycon_control(command)` | `status/connect/start/stop` | `SimpleServiceCallResult` | Joy-Con 基础控制 |
| `joycon_advanced(command,...)` | 高级命令和参数 | `JsonServiceCallResult` | Joy-Con 高级控制 |
| `teach_jog(command,...)` | 示教命令和规划参数 | `JsonServiceCallResult` | 点动或 dry-run |
| `teach_jog_joint(joint_index,delta_deg,dry_run)` | 关节号、角度增量 | `JsonServiceCallResult` | 关节点动快捷方法 |
| `run_teaching_program(...)` | 程序路径或 JSON | `TeachingProgramResult` | 示教程序 Action |
| `embodied_health()` | 无 | `SimpleServiceCallResult` | 具身服务健康检查 |
| `embodied_instruction(instruction,stream)` | 自然语言或 health | `JsonServiceCallResult` | 兼容版具身入口 |
| `embodied_command(command,...)` | functions/actions/run 等 | `JsonServiceCallResult` | 扩展具身入口 |

结果对象说明：

| 类型 | 字段 |
|---|---|
| `TriggerCallResult` | `success`, `message` |
| `DigitalOutputCallResult` | `success`, `message` |
| `GripperCallResult` | `success`, `message` |
| `SimpleServiceCallResult` | `success`, `message` |
| `JsonServiceCallResult` | `success`, `message`, `payload_json` |
| `TeachingProgramResult` | `success`, `message`, `result_json` |
| `TrajectoryExecutionResult` | `success`, `error_code`, `error_string` |
| `InstructionExecutionResult` | `success`, `message` |

已安装示例脚本：

```bash
ros2 pkg prefix horizon_arm_control
```

示例目录：

```text
$(ros2 pkg prefix horizon_arm_control)/share/horizon_arm_control/examples
```

### 12.2 开发建议

写 Python SDK 脚本时，推荐顺序：

1. `wait_until_ready()`。
2. `enable()`。
3. 每次真实动作后 `sleep 3`。
4. 视觉、跟随、示教、具身功能拆成独立函数。
5. 结束时调用 `disable()` 或 `emergency_stop()`。

## 13. 验收接口覆盖

一条命令验收会覆盖 ROS2 运行时、SDK 导入、服务和 Action 就绪、真实运动、夹爪、视觉 wrapper、跟随 wrapper、Joy-Con wrapper、示教 wrapper、具身 wrapper、报告生成。

实机验收示例：

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

其中 `SKIP` 表示按参数跳过相机或 IO 实采，不等于失败；`FAIL=0` 才能作为当前验收通过结论。
