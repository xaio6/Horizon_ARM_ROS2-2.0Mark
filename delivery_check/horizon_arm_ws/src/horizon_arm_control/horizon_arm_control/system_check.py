from __future__ import annotations

import dataclasses
import datetime as dt
import html
import importlib
import json
import math
import os
import time
from pathlib import Path
from typing import Any, Callable, List, Optional

from control_msgs.action import FollowJointTrajectory
from horizon_arm_interfaces.action import RunInstruction, TeachingProgram
from horizon_arm_interfaces.msg import ArmStatus
from horizon_arm_interfaces.srv import (
    DetectTarget,
    EmbodiedCommand,
    EmbodiedInstruction,
    FollowGraspControl,
    FollowTarget,
    JoyconAdvancedControl,
    JoyconControl,
    PickHSV,
    SetDigitalOutput,
    SetGripperState,
    TeachJog,
    VisionConfig,
    VisualGrasp,
    VisualGraspEx,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from .common import prepare_sdk_import


@dataclasses.dataclass
class CheckResult:
    category: str
    name: str
    status: str
    detail: str
    recommendation: str = ""
    context: str = ""


class SystemCheckNode(Node):
    """Unified ROS2 runtime and Linux SDK self-check."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_system_check")

        self.declare_parameter("sdk_root", "")
        self.declare_parameter("report_dir", "")
        self.declare_parameter("acceptance_profile", "custom")
        self.declare_parameter("allow_hardware_side_effects", False)
        self.declare_parameter("status_timeout_sec", 4.0)
        self.declare_parameter("joint_state_timeout_sec", 4.0)
        self.declare_parameter("service_timeout_sec", 2.0)
        self.declare_parameter("action_timeout_sec", 4.0)
        self.declare_parameter("live_step_delay_sec", 0.0)
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("camera_hardware_available", True)
        self.declare_parameter("instruction_smoke_test", True)
        self.declare_parameter("preset_test_enabled", False)
        self.declare_parameter("preset_name", "home_position")
        self.declare_parameter("single_axis_motion_test_enabled", False)
        self.declare_parameter("multi_axis_motion_test_enabled", False)
        self.declare_parameter(
            "single_axis_target_deg", [0.0, 5.0, 0.0, 0.0, 0.0, 0.0]
        )
        self.declare_parameter(
            "multi_axis_target_deg", [5.0, -5.0, 0.0, 0.0, 0.0, 0.0]
        )
        self.declare_parameter("motion_duration_sec", 2.0)
        self.declare_parameter("io_hardware_available", True)
        self.declare_parameter("digital_output_test_enabled", False)
        self.declare_parameter("digital_output_channel", 0)
        self.declare_parameter("digital_output_state", True)
        self.declare_parameter("gripper_test_enabled", False)
        self.declare_parameter("gripper_open_state", True)
        self.declare_parameter("gripper_current_ma", 1200)
        self.declare_parameter("visual_grasp_health_test_enabled", False)
        self.declare_parameter("follow_grasp_health_test_enabled", False)
        self.declare_parameter("joycon_status_test_enabled", False)
        self.declare_parameter("embodied_health_test_enabled", False)
        self.declare_parameter("vision_config_test_enabled", False)
        self.declare_parameter("vision_config_pipeline", "hsv")
        self.declare_parameter("vision_config_target_class", "red_block")
        self.declare_parameter("vision_config_conf_thres", 0.5)
        self.declare_parameter("vision_config_iou_thres", 0.45)
        self.declare_parameter("vision_config_interval_sec", 0.2)
        self.declare_parameter("vision_config_hsv", [0, 12, 80, 255, 60, 255])
        self.declare_parameter("vision_config_depth_range_m", [0.08, 0.80])
        self.declare_parameter("vision_config_pixel_to_mm_scale", 1.0)
        self.declare_parameter("pick_hsv_test_enabled", False)
        self.declare_parameter("pick_hsv_u", 320.0)
        self.declare_parameter("pick_hsv_v", 240.0)
        self.declare_parameter("pick_hsv_window_size", 7)
        self.declare_parameter("pick_hsv_use_depth_filter", False)
        self.declare_parameter("pick_hsv_depth_range_m", [0.08, 0.80])
        self.declare_parameter("detect_target_test_enabled", False)
        self.declare_parameter("detect_target_pipeline", "hsv")
        self.declare_parameter("detect_target_target_class", "red_block")
        self.declare_parameter("detect_target_conf_thres", 0.5)
        self.declare_parameter("detect_target_use_hsv", True)
        self.declare_parameter("detect_target_use_depth", False)
        self.declare_parameter("detect_target_depth_range_m", [0.08, 0.80])
        self.declare_parameter("visual_grasp_ex_test_enabled", False)
        self.declare_parameter("visual_grasp_ex_mode", "click")
        self.declare_parameter("visual_grasp_ex_pipeline", "click")
        self.declare_parameter("visual_grasp_ex_target_class", "")
        self.declare_parameter("visual_grasp_ex_use_depth", True)
        self.declare_parameter("visual_grasp_ex_u", 320.0)
        self.declare_parameter("visual_grasp_ex_v", 240.0)
        self.declare_parameter("visual_grasp_ex_bbox", [240.0, 160.0, 360.0, 280.0])
        self.declare_parameter("visual_grasp_ex_pre_grasp_rpy", [0.0, 0.0, 0.0])
        self.declare_parameter("visual_grasp_ex_approach_height_m", 0.08)
        self.declare_parameter("visual_grasp_ex_grasp_depth_m", 0.02)
        self.declare_parameter("visual_grasp_ex_z_offset_m", 0.0)
        self.declare_parameter("follow_target_status_test_enabled", False)
        self.declare_parameter("follow_target_set_target_test_enabled", False)
        self.declare_parameter("follow_target_mode", "manual")
        self.declare_parameter("follow_target_pipeline", "manual")
        self.declare_parameter("follow_target_target_class", "person")
        self.declare_parameter("follow_target_conf_thres", 0.5)
        self.declare_parameter("follow_target_interval_sec", 0.2)
        self.declare_parameter("follow_target_follow_distance_m", 0.35)
        self.declare_parameter("follow_target_deadband_px", 24.0)
        self.declare_parameter("follow_target_max_linear_speed", 0.08)
        self.declare_parameter("follow_target_max_angular_speed", 0.35)
        self.declare_parameter("follow_target_use_depth", False)
        self.declare_parameter("follow_target_auto_grasp", False)
        self.declare_parameter("follow_target_bbox", [240.0, 160.0, 360.0, 280.0])
        self.declare_parameter("teach_jog_joint_test_enabled", False)
        self.declare_parameter("teach_jog_cartesian_test_enabled", False)
        self.declare_parameter("teach_jog_joint_index", 2)
        self.declare_parameter("teach_jog_joint_delta_deg", 5.0)
        self.declare_parameter("teach_jog_cartesian_command", "base_translate")
        self.declare_parameter("teach_jog_cartesian_frame", "base")
        self.declare_parameter("teach_jog_cartesian_axis", "x")
        self.declare_parameter("teach_jog_cartesian_delta", 10.0)
        self.declare_parameter("teach_jog_interpolation_type", "cartesian")
        self.declare_parameter("teach_jog_linear_velocity", 150.0)
        self.declare_parameter("teach_jog_angular_velocity", 90.0)
        self.declare_parameter("teaching_program_validate_test_enabled", False)
        self.declare_parameter(
            "teaching_program_validate_payload",
            json.dumps(
                {
                    "name": "system_check_demo",
                    "points": [{"joint_angles": [0, 0, 0, 0, 0, 0]}],
                },
                ensure_ascii=True,
            ),
        )
        self.declare_parameter("embodied_functions_test_enabled", False)
        self.declare_parameter("embodied_actions_test_enabled", False)
        self.declare_parameter("instantiate_embodied_sdk", False)
        self.declare_parameter("instantiate_horizon_sdk", False)

        self._results: List[CheckResult] = []
        self._status_msg: Optional[ArmStatus] = None
        self._joint_state_msg: Optional[JointState] = None

        self.create_subscription(ArmStatus, "/horizon_arm/status", self._on_status, 10)
        self.create_subscription(
            JointState, "/horizon_arm/joint_states", self._on_joint_state, 10
        )

        self._enable_client = self.create_client(Trigger, "/horizon_arm/enable")
        self._disable_client = self.create_client(Trigger, "/horizon_arm/disable")
        self._estop_client = self.create_client(
            Trigger, "/horizon_arm/emergency_stop"
        )
        self._do_client = self.create_client(
            SetDigitalOutput, "/horizon_arm/set_digital_output"
        )
        self._gripper_client = self.create_client(
            SetGripperState, "/horizon_arm/set_gripper_state"
        )
        self._visual_grasp_client = self.create_client(
            VisualGrasp, "/horizon_arm/visual_grasp"
        )
        self._visual_grasp_ex_client = self.create_client(
            VisualGraspEx, "/horizon_arm/visual_grasp_ex"
        )
        self._vision_config_client = self.create_client(
            VisionConfig, "/horizon_arm/vision_config"
        )
        self._pick_hsv_client = self.create_client(PickHSV, "/horizon_arm/pick_hsv")
        self._detect_target_client = self.create_client(
            DetectTarget, "/horizon_arm/detect_target"
        )
        self._follow_grasp_client = self.create_client(
            FollowGraspControl, "/horizon_arm/follow_grasp_control"
        )
        self._follow_target_client = self.create_client(
            FollowTarget, "/horizon_arm/follow_target"
        )
        self._joycon_client = self.create_client(
            JoyconControl, "/horizon_arm/joycon_control"
        )
        self._joycon_advanced_client = self.create_client(
            JoyconAdvancedControl, "/horizon_arm/joycon_advanced_control"
        )
        self._teach_jog_client = self.create_client(TeachJog, "/horizon_arm/teach_jog")
        self._embodied_client = self.create_client(
            EmbodiedInstruction, "/horizon_arm/embodied_instruction"
        )
        self._embodied_command_client = self.create_client(
            EmbodiedCommand, "/horizon_arm/embodied_command"
        )
        self._traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/horizon_arm_controller/follow_joint_trajectory",
        )
        self._instruction_client = ActionClient(
            self,
            RunInstruction,
            "/horizon_arm/run_instruction",
        )
        self._teaching_program_client = ActionClient(
            self,
            TeachingProgram,
            "/horizon_arm/teaching_program",
        )

    def run(self) -> int:
        prepare_sdk_import(str(self.get_parameter("sdk_root").value))
        self._apply_acceptance_profile()
        self._check_ros_runtime()
        self._check_sdk_surface()
        self._check_optional_live_tests()
        self._write_reports()
        self._print_summary()
        return 0 if not any(item.status == "FAIL" for item in self._results) else 1

    def _apply_acceptance_profile(self) -> None:
        profile = str(self.get_parameter("acceptance_profile").value).strip().lower()
        allow_hardware = bool(self.get_parameter("allow_hardware_side_effects").value)
        if profile in ("", "custom"):
            return

        always_enable = [
            "vision_config_test_enabled",
            "visual_grasp_ex_test_enabled",
            "follow_target_status_test_enabled",
            "follow_target_set_target_test_enabled",
            "teach_jog_joint_test_enabled",
            "teach_jog_cartesian_test_enabled",
            "teaching_program_validate_test_enabled",
        ]
        for name in always_enable:
            self._set_bool_parameter(name, True)

        if profile in ("full", "full_acceptance", "comprehensive"):
            for name in [
                "visual_grasp_health_test_enabled",
                "follow_grasp_health_test_enabled",
                "joycon_status_test_enabled",
                "embodied_health_test_enabled",
                "embodied_functions_test_enabled",
                "embodied_actions_test_enabled",
            ]:
                self._set_bool_parameter(name, True)
            self._set_bool_parameter(
                "pick_hsv_test_enabled", self._camera_hardware_available()
            )
            self._set_bool_parameter(
                "detect_target_test_enabled", self._camera_hardware_available()
            )

        if allow_hardware and profile in (
            "full",
            "full_acceptance",
            "comprehensive",
            "motion",
        ):
            for name in [
                "instruction_smoke_test",
                "preset_test_enabled",
                "single_axis_motion_test_enabled",
                "multi_axis_motion_test_enabled",
                "gripper_test_enabled",
            ]:
                self._set_bool_parameter(name, True)
            self._set_bool_parameter(
                "digital_output_test_enabled", self._io_hardware_available()
            )
        else:
            for name in [
                "instruction_smoke_test",
                "preset_test_enabled",
                "single_axis_motion_test_enabled",
                "multi_axis_motion_test_enabled",
                "digital_output_test_enabled",
                "gripper_test_enabled",
            ]:
                self._set_bool_parameter(name, False)

    def _set_bool_parameter(self, name: str, value: bool) -> None:
        self.set_parameters([Parameter(name=name, value=bool(value))])

    def _io_hardware_available(self) -> bool:
        return bool(self.get_parameter("io_hardware_available").value)

    def _camera_hardware_available(self) -> bool:
        return bool(self.get_parameter("camera_hardware_available").value)

    def _check_ros_runtime(self) -> None:
        self._wait_for_message(
            name="/horizon_arm/status",
            getter=lambda: self._status_msg,
            timeout_sec=float(self.get_parameter("status_timeout_sec").value),
            detail_builder=self._format_status_detail,
        )
        self._wait_for_message(
            name="/horizon_arm/joint_states",
            getter=lambda: self._joint_state_msg,
            timeout_sec=float(self.get_parameter("joint_state_timeout_sec").value),
            detail_builder=self._format_joint_state_detail,
        )

        self._check_service(self._enable_client, "/horizon_arm/enable")
        self._check_service(self._disable_client, "/horizon_arm/disable")
        self._check_service(self._estop_client, "/horizon_arm/emergency_stop")
        self._check_service(
            self._do_client,
            "/horizon_arm/set_digital_output",
            optional_when=not self._io_hardware_available(),
            optional_reason=(
                "当前环境未接入 IO 硬件，数字输出服务不作为运行时阻塞项。"
            ),
        )
        self._check_service(self._gripper_client, "/horizon_arm/set_gripper_state")
        self._check_service(self._visual_grasp_client, "/horizon_arm/visual_grasp")
        self._check_service(
            self._visual_grasp_ex_client, "/horizon_arm/visual_grasp_ex"
        )
        self._check_service(self._vision_config_client, "/horizon_arm/vision_config")
        self._check_service(self._pick_hsv_client, "/horizon_arm/pick_hsv")
        self._check_service(self._detect_target_client, "/horizon_arm/detect_target")
        self._check_service(
            self._follow_grasp_client, "/horizon_arm/follow_grasp_control"
        )
        self._check_service(self._follow_target_client, "/horizon_arm/follow_target")
        self._check_service(self._joycon_client, "/horizon_arm/joycon_control")
        self._check_service(
            self._joycon_advanced_client, "/horizon_arm/joycon_advanced_control"
        )
        self._check_service(self._teach_jog_client, "/horizon_arm/teach_jog")
        self._check_service(
            self._embodied_client, "/horizon_arm/embodied_instruction"
        )
        self._check_service(
            self._embodied_command_client, "/horizon_arm/embodied_command"
        )
        self._check_action(
            self._traj_client, "/horizon_arm_controller/follow_joint_trajectory"
        )
        self._check_action(self._instruction_client, "/horizon_arm/run_instruction")
        self._check_action(
            self._teaching_program_client, "/horizon_arm/teaching_program"
        )

    def _check_sdk_surface(self) -> None:
        camera_id = int(self.get_parameter("camera_id").value)
        self._check_sdk_import("Embodied_SDK", category="sdk")
        self._check_sdk_attr("Embodied_SDK", "MotionSDK", instantiate=lambda cls: cls())
        self._check_sdk_attr(
            "Embodied_SDK",
            "VisualGraspSDK",
            instantiate=lambda cls: cls(camera_id=camera_id),
        )
        self._check_sdk_attr(
            "Embodied_SDK",
            "FollowGraspSDK",
            instantiate=lambda cls: cls(camera_id=camera_id),
        )
        self._check_sdk_attr("Embodied_SDK", "JoyconSDK", instantiate=lambda cls: cls())
        self._check_sdk_attr("Embodied_SDK", "IOSDK", instantiate=lambda cls: cls())
        self._check_sdk_attr("Embodied_SDK", "ZDTGripperSDK")
        self._check_sdk_attr("Embodied_SDK", "HorizonArmSDK")
        self._check_sdk_attr("Embodied_SDK", "AISDK")
        self._check_sdk_attr("Embodied_SDK", "DepthEstimationSDK")
        self._check_sdk_attr("Embodied_SDK", "EmbodiedSDK")

        if bool(self.get_parameter("instantiate_embodied_sdk").value):
            self._check_sdk_attr(
                "Embodied_SDK",
                "EmbodiedSDK",
                name_override="EmbodiedSDK live init",
                instantiate=lambda cls: cls(),
            )
        else:
            self._skip(
                "sdk",
                "EmbodiedSDK live init",
                "Disabled by parameter.",
                recommendation=(
                    "Enable instantiate_embodied_sdk:=true only in a fully configured AI environment."
                ),
            )

        if bool(self.get_parameter("instantiate_horizon_sdk").value):
            self._check_sdk_attr(
                "Embodied_SDK",
                "HorizonArmSDK",
                name_override="HorizonArmSDK live init",
                instantiate=lambda cls: cls(),
            )
        else:
            self._skip(
                "sdk",
                "HorizonArmSDK live init",
                "Disabled by parameter.",
                recommendation=(
                    "Enable instantiate_horizon_sdk:=true after preparing a controlled test harness."
                ),
            )

        self._results.extend(
            [
                CheckResult(
                    category="capability",
                    name="Gripper ROS2 wrapper",
                    status="PASS",
                    detail=(
                        "Gripper ROS2 service wrapper is exposed via "
                        "/horizon_arm/set_gripper_state and RunInstruction."
                    ),
                    recommendation=(
                        "Use the gripper ROS2 service or RunInstruction for quick Linux verification."
                    ),
                ),
                CheckResult(
                    category="capability",
                    name="Vision/Follow ROS2 wrapper",
                    status="PASS",
                    detail=(
                        "Visual grasp, enhanced visual grasp, vision config, target "
                        "detection, follow grasp, and follow target wrappers are exposed."
                    ),
                    recommendation=(
                        "Use the wrapper services for health checks first, then enable real camera tasks during integration."
                    ),
                ),
                CheckResult(
                    category="capability",
                    name="Joycon ROS2 wrapper",
                    status="PASS",
                    detail=(
                        "Joycon wrappers are exposed via /horizon_arm/joycon_control "
                        "and /horizon_arm/joycon_advanced_control."
                    ),
                    recommendation=(
                        "Use the status command by default and connect/start only when the controller is present."
                    ),
                ),
                CheckResult(
                    category="capability",
                    name="Teaching ROS2 wrapper",
                    status="PASS",
                    detail=(
                        "Teaching jog and teaching program wrappers are exposed via "
                        "/horizon_arm/teach_jog and /horizon_arm/teaching_program."
                    ),
                    recommendation=(
                        "Use dry_run and validate before running a saved teaching program on real hardware."
                    ),
                ),
                CheckResult(
                    category="capability",
                    name="Embodied AI ROS2 wrapper",
                    status="PASS",
                    detail=(
                        "Embodied AI wrappers are exposed via /horizon_arm/embodied_instruction "
                        "and /horizon_arm/embodied_command."
                    ),
                    recommendation=(
                        "Use the health command by default and enable real NL control only after AI credentials are configured."
                    ),
                ),
            ]
        )

    def _check_optional_live_tests(self) -> None:
        if bool(self.get_parameter("instruction_smoke_test").value):
            self._test_instruction_smoke("enable", "RunInstruction smoke test")
            self._delay_after_live_step("RunInstruction smoke test")
        else:
            self._skip_live_hardware("RunInstruction smoke test", "机械臂统一指令烟测")

        if bool(self.get_parameter("preset_test_enabled").value):
            preset_name = str(self.get_parameter("preset_name").value)
            self._test_instruction_smoke(
                f"preset:{preset_name}",
                f"Preset motion test ({preset_name})",
            )
            self._delay_after_live_step(f"Preset motion test ({preset_name})")
        else:
            self._skip_live_hardware("Preset motion test", "机械臂预设动作")

        if bool(self.get_parameter("single_axis_motion_test_enabled").value):
            joints = list(self.get_parameter("single_axis_target_deg").value)
            self._test_instruction_smoke(
                json.dumps(
                    {
                        "command": "move_joints_deg",
                        "joints": joints,
                        "duration": float(self.get_parameter("motion_duration_sec").value),
                    },
                    ensure_ascii=True,
                ),
                "Single-axis motion test",
            )
            self._delay_after_live_step("Single-axis motion test")
        else:
            self._skip_live_hardware("Single-axis motion test", "机械臂单轴运动")

        if bool(self.get_parameter("multi_axis_motion_test_enabled").value):
            joints = list(self.get_parameter("multi_axis_target_deg").value)
            self._test_instruction_smoke(
                json.dumps(
                    {
                        "command": "move_joints_deg",
                        "joints": joints,
                        "duration": float(self.get_parameter("motion_duration_sec").value),
                    },
                    ensure_ascii=True,
                ),
                "Multi-axis motion test",
            )
            self._delay_after_live_step("Multi-axis motion test")
        else:
            self._skip_live_hardware("Multi-axis motion test", "机械臂多轴运动")

        if bool(self.get_parameter("digital_output_test_enabled").value):
            self._test_digital_output()
            self._delay_after_live_step("Digital output live test")
        else:
            if self._io_hardware_available():
                self._skip(
                    "live",
                    "Digital output live test",
                    self._not_performed_detail("数字输出"),
                    recommendation=(
                        "如需验收 IO 实机输出，请接好外部 IO 模块并使用实机测试命令重新执行。"
                    ),
                )
            else:
                self._skip(
                    "live",
                    "Digital output live test",
                    "Skipped physical IO output because no IO hardware is installed in the current environment. "
                    "ROS2 service readiness is still covered by /horizon_arm/set_digital_output.",
                    recommendation=(
                        "No real IO module is present in this environment. Treat the ROS2 service readiness check "
                        "as the logical interface validation and only enable live IO output after hardware is added."
                    ),
                )

        if bool(self.get_parameter("gripper_test_enabled").value):
            self._test_gripper()
            self._delay_after_live_step("Gripper close/open test")
        else:
            self._skip_live_hardware("Gripper close test", "夹爪闭合")
            self._skip_live_hardware("Gripper open test", "夹爪张开")

        if bool(self.get_parameter("visual_grasp_health_test_enabled").value):
            self._test_visual_grasp_health()
        else:
            self._skip("live", "Visual grasp health test", "Disabled by parameter.")

        if bool(self.get_parameter("follow_grasp_health_test_enabled").value):
            self._test_follow_grasp_health()
        else:
            self._skip("live", "Follow grasp health test", "Disabled by parameter.")

        if bool(self.get_parameter("joycon_status_test_enabled").value):
            self._test_joycon_status()
        else:
            self._skip("live", "Joycon status test", "Disabled by parameter.")

        if bool(self.get_parameter("embodied_health_test_enabled").value):
            self._test_embodied_health()
        else:
            self._skip("live", "Embodied health test", "Disabled by parameter.")

        if bool(self.get_parameter("vision_config_test_enabled").value):
            self._test_vision_config()
        else:
            self._skip("live", "Vision config update test", "Disabled by parameter.")

        if bool(self.get_parameter("pick_hsv_test_enabled").value):
            self._test_pick_hsv()
        else:
            self._skip("live", "Pick HSV sample test", "Disabled by parameter.")

        if bool(self.get_parameter("detect_target_test_enabled").value):
            self._test_detect_target()
        else:
            self._skip("live", "Detect target test", "Disabled by parameter.")

        if bool(self.get_parameter("visual_grasp_ex_test_enabled").value):
            self._test_visual_grasp_ex()
        else:
            self._skip("live", "Visual grasp ex dry-run test", "Disabled by parameter.")

        if bool(self.get_parameter("follow_target_status_test_enabled").value):
            self._test_follow_target_status()
        else:
            self._skip("live", "Follow target status test", "Disabled by parameter.")

        if bool(self.get_parameter("follow_target_set_target_test_enabled").value):
            self._test_follow_target_set_target()
        else:
            self._skip("live", "Follow target manual target test", "Disabled by parameter.")

        if bool(self.get_parameter("teach_jog_joint_test_enabled").value):
            self._test_teach_jog_joint()
        else:
            self._skip("live", "Teach jog joint dry-run test", "Disabled by parameter.")

        if bool(self.get_parameter("teach_jog_cartesian_test_enabled").value):
            self._test_teach_jog_cartesian()
        else:
            self._skip("live", "Teach jog cartesian dry-run test", "Disabled by parameter.")

        if bool(self.get_parameter("teaching_program_validate_test_enabled").value):
            self._test_teaching_program_validate()
        else:
            self._skip("live", "Teaching program validate test", "Disabled by parameter.")

        if bool(self.get_parameter("embodied_functions_test_enabled").value):
            self._test_embodied_functions()
        else:
            self._skip("live", "Embodied functions test", "Disabled by parameter.")

        if bool(self.get_parameter("embodied_actions_test_enabled").value):
            self._test_embodied_actions()
        else:
            self._skip("live", "Embodied actions test", "Disabled by parameter.")

    def _not_performed_detail(self, capability: str) -> str:
        return (
            f"{capability} 未进行实机测试：本次命令未开启 allow_hardware_side_effects，"
            "脚本不会连接机械臂/夹爪执行真实动作；仅保留 ROS2 接口、SDK 导入和逻辑链路检查。"
        )

    def _skip_live_hardware(self, result_name: str, capability: str) -> None:
        if bool(self.get_parameter("allow_hardware_side_effects").value):
            self._skip("live", result_name, "Disabled by parameter.")
            return
        self._skip(
            "live",
            result_name,
            self._not_performed_detail(capability),
            recommendation=(
                "如需最终实机验收，请确认现场安全、机械臂和夹爪已连接后，"
                "使用 --real-hardware 重新执行一条命令验收。"
            ),
        )

    def _live_step_delay_sec(self) -> float:
        if not bool(self.get_parameter("allow_hardware_side_effects").value):
            return 0.0
        return max(0.0, float(self.get_parameter("live_step_delay_sec").value))

    def _delay_after_live_step(self, label: str) -> None:
        delay_sec = self._live_step_delay_sec()
        if delay_sec <= 0.0:
            return
        self.get_logger().info(
            f"Waiting {delay_sec:.2f}s after {label} for hardware to settle."
        )
        time.sleep(delay_sec)

    def _wait_for_message(
        self,
        *,
        name: str,
        getter: Callable[[], Any],
        timeout_sec: float,
        detail_builder: Callable[[Any], str],
    ) -> None:
        deadline = time.time() + max(0.1, timeout_sec)
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            message = getter()
            if message is not None:
                self._results.append(
                    CheckResult(
                        category="ros_runtime",
                        name=name,
                        status="PASS",
                        detail=detail_builder(message),
                    )
                )
                return
        self._results.append(
            CheckResult(
                category="ros_runtime",
                name=name,
                status="FAIL",
                detail=f"No message received within {timeout_sec:.1f}s.",
            )
        )

    def _check_service(
        self,
        client,
        service_name: str,
        *,
        optional_when: bool = False,
        optional_reason: str = "",
    ) -> None:
        timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        ready = client.wait_for_service(timeout_sec=timeout_sec)
        if (not ready) and optional_when:
            self._results.append(
                CheckResult(
                    category="ros_runtime",
                    name=service_name,
                    status="SKIP",
                    detail=optional_reason
                    or f"Service was not ready within {timeout_sec:.1f}s, but this item is optional in the current environment.",
                    recommendation="在接入对应硬件后，再重新检查该服务运行时状态。",
                )
            )
            return
        self._results.append(
            CheckResult(
                category="ros_runtime",
                name=service_name,
                status="PASS" if ready else "FAIL",
                detail=(
                    f"Service became ready within {timeout_sec:.1f}s."
                    if ready
                    else f"Service was not ready within {timeout_sec:.1f}s."
                ),
            )
        )

    def _check_action(self, client: ActionClient, action_name: str) -> None:
        timeout_sec = float(self.get_parameter("action_timeout_sec").value)
        ready = client.wait_for_server(timeout_sec=timeout_sec)
        self._results.append(
            CheckResult(
                category="ros_runtime",
                name=action_name,
                status="PASS" if ready else "FAIL",
                detail=(
                    f"Action server became ready within {timeout_sec:.1f}s."
                    if ready
                    else f"Action server was not ready within {timeout_sec:.1f}s."
                ),
            )
        )

    def _check_sdk_import(self, module_name: str, *, category: str) -> None:
        try:
            importlib.import_module(module_name)
            self._results.append(
                CheckResult(
                    category=category,
                    name=module_name,
                    status="PASS",
                    detail="Module import succeeded.",
                )
            )
        except Exception as exc:
            self._results.append(
                CheckResult(
                    category=category,
                    name=module_name,
                    status="FAIL",
                    detail=f"Import failed: {exc}",
                )
            )

    def _check_sdk_attr(
        self,
        module_name: str,
        attr_name: str,
        *,
        instantiate: Optional[Callable[[Any], Any]] = None,
        name_override: Optional[str] = None,
    ) -> None:
        display_name = name_override or attr_name
        try:
            module = importlib.import_module(module_name)
            value = getattr(module, attr_name)
            detail = "Attribute import succeeded."
            if instantiate is not None:
                instance = instantiate(value)
                detail = f"Instantiation succeeded: {type(instance).__name__}."
            self._results.append(
                CheckResult(
                    category="sdk",
                    name=display_name,
                    status="PASS",
                    detail=detail,
                )
            )
        except Exception as exc:
            self._results.append(
                CheckResult(
                    category="sdk",
                    name=display_name,
                    status="FAIL",
                    detail=f"Check failed: {exc}",
                )
            )

    def _test_instruction_smoke(self, instruction: str, label: str) -> None:
        timeout_sec = max(
            5.0, float(self.get_parameter("action_timeout_sec").value) * 3.0
        )
        if not self._instruction_client.wait_for_server(timeout_sec=timeout_sec):
            self._results.append(
                CheckResult(
                    category="live",
                    name=label,
                    status="FAIL",
                    detail="RunInstruction action server is not ready.",
                )
            )
            return

        goal = RunInstruction.Goal()
        goal.instruction = instruction
        try:
            send_future = self._instruction_client.send_goal_async(goal)
            goal_handle = self._wait_for_future(send_future, timeout_sec=timeout_sec)
            if not goal_handle.accepted:
                self._results.append(
                    CheckResult(
                        category="live",
                        name=label,
                        status="FAIL",
                        detail="Goal was rejected.",
                    )
                )
                return
            result_future = goal_handle.get_result_async()
            wrapped = self._wait_for_future(result_future, timeout_sec=timeout_sec)
            result = wrapped.result
            self._results.append(
                CheckResult(
                    category="live",
                    name=label,
                    status="PASS" if result.success else "FAIL",
                    detail=str(result.message),
                )
            )
        except Exception as exc:
            self._results.append(
                CheckResult(
                    category="live",
                    name=label,
                    status="FAIL",
                    detail=f"Execution failed: {exc}",
                )
            )

    def _test_digital_output(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        if not self._do_client.wait_for_service(timeout_sec=timeout_sec):
            self._results.append(
                CheckResult(
                    category="live",
                    name="Digital output live test",
                    status="FAIL",
                    detail="Digital output service is not ready.",
                )
            )
            return

        request = SetDigitalOutput.Request()
        request.channel = int(self.get_parameter("digital_output_channel").value)
        request.state = bool(self.get_parameter("digital_output_state").value)
        try:
            future = self._do_client.call_async(request)
            response = self._wait_for_future(future, timeout_sec=timeout_sec)
            self._results.append(
                CheckResult(
                    category="live",
                    name="Digital output live test",
                    status="PASS" if response.success else "FAIL",
                    detail=str(response.message),
                )
            )
        except Exception as exc:
            self._results.append(
                CheckResult(
                    category="live",
                    name="Digital output live test",
                    status="FAIL",
                    detail=f"Service call failed: {exc}",
                )
            )

    def _test_gripper(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        if not self._gripper_client.wait_for_service(timeout_sec=timeout_sec):
            self._results.append(
                CheckResult(
                    category="live",
                    name="Gripper close test",
                    status="FAIL",
                    detail="Gripper service is not ready.",
                )
            )
            self._results.append(
                CheckResult(
                    category="live",
                    name="Gripper open test",
                    status="FAIL",
                    detail="Gripper service is not ready.",
                )
            )
            return

        current_ma = max(0, int(self.get_parameter("gripper_current_ma").value))
        close_request = SetGripperState.Request()
        close_request.open = False
        close_request.current_ma = current_ma
        open_request = SetGripperState.Request()
        open_request.open = True
        open_request.current_ma = current_ma

        self._call_simple_service(
            self._gripper_client,
            close_request,
            result_name="Gripper close test",
            timeout_sec=timeout_sec,
        )
        self._delay_after_live_step("Gripper close test")
        self._call_simple_service(
            self._gripper_client,
            open_request,
            result_name="Gripper open test",
            timeout_sec=timeout_sec,
        )

    def _test_visual_grasp_health(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        request = VisualGrasp.Request()
        request.dry_run = True
        self._call_simple_service(
            self._visual_grasp_client,
            request,
            result_name="Visual grasp health test",
            timeout_sec=timeout_sec,
            not_ready_detail="Visual grasp service is not ready.",
        )

    def _test_follow_grasp_health(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        request = FollowGraspControl.Request()
        request.command = "health"
        self._call_simple_service(
            self._follow_grasp_client,
            request,
            result_name="Follow grasp health test",
            timeout_sec=timeout_sec,
            not_ready_detail="Follow grasp service is not ready.",
        )

    def _test_joycon_status(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        request = JoyconControl.Request()
        request.command = "status"
        self._call_simple_service(
            self._joycon_client,
            request,
            result_name="Joycon status test",
            timeout_sec=timeout_sec,
            not_ready_detail="Joycon control service is not ready.",
        )

    def _test_embodied_health(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        request = EmbodiedInstruction.Request()
        request.command = "health"
        request.stream = False
        self._call_simple_service(
            self._embodied_client,
            request,
            result_name="Embodied health test",
            timeout_sec=timeout_sec,
            not_ready_detail="Embodied instruction service is not ready.",
        )

    def _test_vision_config(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        hsv = self._parameter_list("vision_config_hsv", expected_len=6, cast=int)
        depth_range = self._parameter_list(
            "vision_config_depth_range_m",
            expected_len=2,
            cast=float,
        )
        request = VisionConfig.Request()
        request.command = "set"
        request.pipeline = str(self.get_parameter("vision_config_pipeline").value)
        request.target_class = str(
            self.get_parameter("vision_config_target_class").value
        )
        request.conf_thres = float(self.get_parameter("vision_config_conf_thres").value)
        request.iou_thres = float(self.get_parameter("vision_config_iou_thres").value)
        request.interval_sec = float(
            self.get_parameter("vision_config_interval_sec").value
        )
        request.hsv_h_min = hsv[0]
        request.hsv_h_max = hsv[1]
        request.hsv_s_min = hsv[2]
        request.hsv_s_max = hsv[3]
        request.hsv_v_min = hsv[4]
        request.hsv_v_max = hsv[5]
        request.depth_min_m = depth_range[0]
        request.depth_max_m = depth_range[1]
        request.pixel_to_mm_scale = float(
            self.get_parameter("vision_config_pixel_to_mm_scale").value
        )
        request.options_json = json.dumps({"source": "system_check"}, ensure_ascii=True)
        self._call_service_with_detail(
            self._vision_config_client,
            request,
            result_name="Vision config update test",
            timeout_sec=timeout_sec,
            not_ready_detail="Vision config service is not ready.",
            detail_builder=lambda response: self._json_detail(
                message=getattr(response, "message", ""),
                config_json=getattr(response, "config_json", "{}"),
            ),
        )

    def _test_pick_hsv(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        depth_range = self._parameter_list(
            "pick_hsv_depth_range_m",
            expected_len=2,
            cast=float,
        )
        request = PickHSV.Request()
        request.u = float(self.get_parameter("pick_hsv_u").value)
        request.v = float(self.get_parameter("pick_hsv_v").value)
        request.window_size = int(self.get_parameter("pick_hsv_window_size").value)
        request.use_depth_filter = bool(
            self.get_parameter("pick_hsv_use_depth_filter").value
        )
        request.depth_min_m = depth_range[0]
        request.depth_max_m = depth_range[1]
        self._call_service_with_detail(
            self._pick_hsv_client,
            request,
            result_name="Pick HSV sample test",
            timeout_sec=timeout_sec,
            not_ready_detail="Pick HSV service is not ready.",
            detail_builder=lambda response: self._json_detail(
                message=getattr(response, "message", ""),
                h=getattr(response, "h", 0),
                s=getattr(response, "s", 0),
                v=getattr(response, "v", 0),
                h_range=[getattr(response, "h_min", 0), getattr(response, "h_max", 0)],
                s_range=[getattr(response, "s_min", 0), getattr(response, "s_max", 0)],
                v_range=[getattr(response, "v_min", 0), getattr(response, "v_max", 0)],
                depth_m=getattr(response, "depth_m", 0.0),
            ),
            response_evaluator=lambda response: self._camera_optional_result(
                response,
                result_name="Pick HSV sample test",
                detail_builder=lambda item: self._json_detail(
                    message=getattr(item, "message", ""),
                    h=getattr(item, "h", 0),
                    s=getattr(item, "s", 0),
                    v=getattr(item, "v", 0),
                    h_range=[getattr(item, "h_min", 0), getattr(item, "h_max", 0)],
                    s_range=[getattr(item, "s_min", 0), getattr(item, "s_max", 0)],
                    v_range=[getattr(item, "v_min", 0), getattr(item, "v_max", 0)],
                    depth_m=getattr(item, "depth_m", 0.0),
                ),
            ),
        )

    def _test_detect_target(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        depth_range = self._parameter_list(
            "detect_target_depth_range_m",
            expected_len=2,
            cast=float,
        )
        request = DetectTarget.Request()
        request.pipeline = str(self.get_parameter("detect_target_pipeline").value)
        request.target_class = str(
            self.get_parameter("detect_target_target_class").value
        )
        request.conf_thres = float(self.get_parameter("detect_target_conf_thres").value)
        request.use_hsv = bool(self.get_parameter("detect_target_use_hsv").value)
        request.use_depth = bool(self.get_parameter("detect_target_use_depth").value)
        request.depth_min_m = depth_range[0]
        request.depth_max_m = depth_range[1]
        self._call_service_with_detail(
            self._detect_target_client,
            request,
            result_name="Detect target test",
            timeout_sec=timeout_sec,
            not_ready_detail="Detect target service is not ready.",
            detail_builder=lambda response: self._json_detail(
                message=getattr(response, "message", ""),
                count=getattr(response, "count", 0),
                class_names=list(getattr(response, "class_names", [])),
                centers=list(getattr(response, "centers", [])),
                scores=list(getattr(response, "scores", [])),
                depths_m=list(getattr(response, "depths_m", [])),
            ),
            response_evaluator=lambda response: self._camera_optional_result(
                response,
                result_name="Detect target test",
                detail_builder=lambda item: self._json_detail(
                    message=getattr(item, "message", ""),
                    count=getattr(item, "count", 0),
                    class_names=list(getattr(item, "class_names", [])),
                    centers=list(getattr(item, "centers", [])),
                    scores=list(getattr(item, "scores", [])),
                    depths_m=list(getattr(item, "depths_m", [])),
                ),
            ),
        )

    def _test_visual_grasp_ex(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        bbox = self._parameter_list("visual_grasp_ex_bbox", expected_len=4, cast=float)
        rpy = self._parameter_list(
            "visual_grasp_ex_pre_grasp_rpy",
            expected_len=3,
            cast=float,
        )
        request = VisualGraspEx.Request()
        request.mode = str(self.get_parameter("visual_grasp_ex_mode").value)
        request.pipeline = str(self.get_parameter("visual_grasp_ex_pipeline").value)
        request.target_class = str(
            self.get_parameter("visual_grasp_ex_target_class").value
        )
        request.dry_run = True
        request.use_click = request.mode in ("click", "pixel")
        request.use_bbox = request.mode == "bbox"
        request.use_hsv = request.pipeline == "hsv"
        request.use_depth = bool(self.get_parameter("visual_grasp_ex_use_depth").value)
        request.u = float(self.get_parameter("visual_grasp_ex_u").value)
        request.v = float(self.get_parameter("visual_grasp_ex_v").value)
        request.x1 = bbox[0]
        request.y1 = bbox[1]
        request.x2 = bbox[2]
        request.y2 = bbox[3]
        request.z_offset_m = float(self.get_parameter("visual_grasp_ex_z_offset_m").value)
        request.approach_height_m = float(
            self.get_parameter("visual_grasp_ex_approach_height_m").value
        )
        request.grasp_depth_m = float(
            self.get_parameter("visual_grasp_ex_grasp_depth_m").value
        )
        request.pre_grasp_rpy = rpy
        request.options_json = json.dumps({"source": "system_check"}, ensure_ascii=True)
        self._call_service_with_detail(
            self._visual_grasp_ex_client,
            request,
            result_name="Visual grasp ex dry-run test",
            timeout_sec=timeout_sec,
            not_ready_detail="Visual grasp ex service is not ready.",
            detail_builder=lambda response: self._json_detail(
                message=getattr(response, "message", ""),
                target_xyz=list(getattr(response, "target_xyz", [])),
                target_rpy=list(getattr(response, "target_rpy", [])),
                result_json=getattr(response, "result_json", "{}"),
            ),
            response_evaluator=lambda response: self._camera_optional_result(
                response,
                result_name="Visual grasp ex dry-run test",
                detail_builder=lambda item: self._json_detail(
                    message=getattr(item, "message", ""),
                    target_xyz=list(getattr(item, "target_xyz", [])),
                    target_rpy=list(getattr(item, "target_rpy", [])),
                    result_json=getattr(item, "result_json", "{}"),
                ),
            ),
        )

    def _test_follow_target_status(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        request = FollowTarget.Request()
        request.command = "status"
        request.mode = str(self.get_parameter("follow_target_mode").value)
        request.pipeline = str(self.get_parameter("follow_target_pipeline").value)
        request.target_class = str(
            self.get_parameter("follow_target_target_class").value
        )
        self._call_service_with_detail(
            self._follow_target_client,
            request,
            result_name="Follow target status test",
            timeout_sec=timeout_sec,
            not_ready_detail="Follow target service is not ready.",
            detail_builder=lambda response: self._json_detail(
                message=getattr(response, "message", ""),
                running=bool(getattr(response, "running", False)),
                state_json=getattr(response, "state_json", "{}"),
            ),
        )

    def _test_follow_target_set_target(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        bbox = self._parameter_list("follow_target_bbox", expected_len=4, cast=float)
        request = FollowTarget.Request()
        request.command = "set_target"
        request.mode = "manual"
        request.pipeline = str(self.get_parameter("follow_target_pipeline").value)
        request.target_class = str(
            self.get_parameter("follow_target_target_class").value
        )
        request.conf_thres = float(self.get_parameter("follow_target_conf_thres").value)
        request.interval_sec = float(
            self.get_parameter("follow_target_interval_sec").value
        )
        request.follow_distance_m = float(
            self.get_parameter("follow_target_follow_distance_m").value
        )
        request.deadband_px = float(
            self.get_parameter("follow_target_deadband_px").value
        )
        request.max_linear_speed = float(
            self.get_parameter("follow_target_max_linear_speed").value
        )
        request.max_angular_speed = float(
            self.get_parameter("follow_target_max_angular_speed").value
        )
        request.use_depth = bool(self.get_parameter("follow_target_use_depth").value)
        request.auto_grasp = bool(self.get_parameter("follow_target_auto_grasp").value)
        request.options_json = json.dumps(
            {"x1": bbox[0], "y1": bbox[1], "x2": bbox[2], "y2": bbox[3]},
            ensure_ascii=True,
        )
        self._call_service_with_detail(
            self._follow_target_client,
            request,
            result_name="Follow target manual target test",
            timeout_sec=timeout_sec,
            not_ready_detail="Follow target service is not ready.",
            detail_builder=lambda response: self._json_detail(
                message=getattr(response, "message", ""),
                running=bool(getattr(response, "running", False)),
                state_json=getattr(response, "state_json", "{}"),
            ),
            response_evaluator=lambda response: self._camera_optional_result(
                response,
                result_name="Follow target manual target test",
                detail_builder=lambda item: self._json_detail(
                    message=getattr(item, "message", ""),
                    running=bool(getattr(item, "running", False)),
                    state_json=getattr(item, "state_json", "{}"),
                ),
            ),
        )

    def _test_teach_jog_joint(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        request = TeachJog.Request()
        request.command = "joint_jog"
        request.joint_index = int(self.get_parameter("teach_jog_joint_index").value)
        request.delta = float(self.get_parameter("teach_jog_joint_delta_deg").value)
        request.interpolation_type = "joint"
        request.dry_run = True
        self._call_service_with_detail(
            self._teach_jog_client,
            request,
            result_name="Teach jog joint dry-run test",
            timeout_sec=timeout_sec,
            not_ready_detail="Teach jog service is not ready.",
            detail_builder=lambda response: self._json_detail(
                message=getattr(response, "message", ""),
                target_joint_angles=list(getattr(response, "target_joint_angles", [])),
                detail_json=getattr(response, "detail_json", "{}"),
            ),
            response_evaluator=lambda response: self._joint_result(
                response,
                result_name="Teach jog joint dry-run test",
                detail_json=getattr(response, "detail_json", "{}"),
            ),
        )

    def _test_teach_jog_cartesian(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        request = TeachJog.Request()
        request.command = str(self.get_parameter("teach_jog_cartesian_command").value)
        request.frame = str(self.get_parameter("teach_jog_cartesian_frame").value)
        request.axis = str(self.get_parameter("teach_jog_cartesian_axis").value)
        request.delta = float(self.get_parameter("teach_jog_cartesian_delta").value)
        request.interpolation_type = str(
            self.get_parameter("teach_jog_interpolation_type").value
        )
        request.linear_velocity = float(
            self.get_parameter("teach_jog_linear_velocity").value
        )
        request.angular_velocity = float(
            self.get_parameter("teach_jog_angular_velocity").value
        )
        request.dry_run = True
        self._call_service_with_detail(
            self._teach_jog_client,
            request,
            result_name="Teach jog cartesian dry-run test",
            timeout_sec=timeout_sec,
            not_ready_detail="Teach jog service is not ready.",
            detail_builder=lambda response: self._json_detail(
                message=getattr(response, "message", ""),
                target_joint_angles=list(getattr(response, "target_joint_angles", [])),
                detail_json=getattr(response, "detail_json", "{}"),
            ),
            response_evaluator=lambda response: self._joint_result(
                response,
                result_name="Teach jog cartesian dry-run test",
                detail_json=getattr(response, "detail_json", "{}"),
            ),
        )

    def _test_teaching_program_validate(self) -> None:
        timeout_sec = max(
            5.0, float(self.get_parameter("action_timeout_sec").value) * 3.0
        )
        if not self._teaching_program_client.wait_for_server(timeout_sec=timeout_sec):
            self._results.append(
                CheckResult(
                    category="live",
                    name="Teaching program validate test",
                    status="FAIL",
                    detail="Teaching program action server is not ready.",
                )
            )
            return

        goal = TeachingProgram.Goal()
        goal.command = "validate"
        goal.dry_run = True
        goal.program_json = str(
            self.get_parameter("teaching_program_validate_payload").value
        )
        try:
            send_future = self._teaching_program_client.send_goal_async(goal)
            goal_handle = self._wait_for_future(send_future, timeout_sec=timeout_sec)
            if not goal_handle.accepted:
                self._results.append(
                    CheckResult(
                        category="live",
                        name="Teaching program validate test",
                        status="FAIL",
                        detail="Teaching program validation goal was rejected.",
                    )
                )
                return
            result_future = goal_handle.get_result_async()
            wrapped = self._wait_for_future(result_future, timeout_sec=timeout_sec)
            result = wrapped.result
            detail = self._json_detail(
                message=getattr(result, "message", ""),
                result_json=getattr(result, "result_json", "{}"),
            )
            self._results.append(
                CheckResult(
                    category="live",
                    name="Teaching program validate test",
                    status="PASS" if result.success else "FAIL",
                    detail=detail,
                )
            )
        except Exception as exc:
            self._results.append(
                CheckResult(
                    category="live",
                    name="Teaching program validate test",
                    status="FAIL",
                    detail=f"Execution failed: {exc}",
                )
            )

    def _test_embodied_functions(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        request = EmbodiedCommand.Request()
        request.command = "functions"
        request.stream = False
        self._call_service_with_detail(
            self._embodied_command_client,
            request,
            result_name="Embodied functions test",
            timeout_sec=timeout_sec,
            not_ready_detail="Embodied command service is not ready.",
            detail_builder=lambda response: self._json_detail(
                message=getattr(response, "message", ""),
                result_json=getattr(response, "result_json", "{}"),
            ),
        )

    def _test_embodied_actions(self) -> None:
        timeout_sec = max(2.0, float(self.get_parameter("service_timeout_sec").value))
        request = EmbodiedCommand.Request()
        request.command = "actions"
        request.stream = False
        self._call_service_with_detail(
            self._embodied_command_client,
            request,
            result_name="Embodied actions test",
            timeout_sec=timeout_sec,
            not_ready_detail="Embodied command service is not ready.",
            detail_builder=lambda response: self._json_detail(
                message=getattr(response, "message", ""),
                result_json=getattr(response, "result_json", "{}"),
            ),
        )

    def _call_simple_service(
        self,
        client,
        request,
        *,
        result_name: str,
        timeout_sec: float,
        not_ready_detail: str = "Service is not ready.",
        request_context: str = "",
    ) -> None:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self._results.append(
                CheckResult(
                    category="live",
                    name=result_name,
                    status="FAIL",
                    detail=not_ready_detail,
                    context=request_context or self._request_context(request),
                )
            )
            return
        try:
            future = client.call_async(request)
            response = self._wait_for_future(future, timeout_sec=timeout_sec)
            success = bool(getattr(response, "success", True))
            message = str(getattr(response, "message", "completed"))
            self._results.append(
                CheckResult(
                    category="live",
                    name=result_name,
                    status="PASS" if success else "FAIL",
                    detail=message,
                    context=request_context or self._request_context(request),
                )
            )
        except Exception as exc:
            self._results.append(
                CheckResult(
                    category="live",
                    name=result_name,
                    status="FAIL",
                    detail=f"Service call failed: {exc}",
                    context=request_context or self._request_context(request),
                )
            )

    def _call_service_with_detail(
        self,
        client,
        request,
        *,
        result_name: str,
        timeout_sec: float,
        not_ready_detail: str,
        detail_builder: Callable[[Any], str],
        request_context: str = "",
        response_evaluator: Optional[Callable[[Any], CheckResult]] = None,
    ) -> None:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self._results.append(
                CheckResult(
                    category="live",
                    name=result_name,
                    status="FAIL",
                    detail=not_ready_detail,
                    context=request_context,
                )
            )
            return
        try:
            future = client.call_async(request)
            response = self._wait_for_future(future, timeout_sec=timeout_sec)
            if response_evaluator is not None:
                result = response_evaluator(response)
                if not result.context:
                    result.context = request_context or self._request_context(request)
                self._results.append(result)
                return
            success = bool(getattr(response, "success", True))
            self._results.append(
                CheckResult(
                    category="live",
                    name=result_name,
                    status="PASS" if success else "FAIL",
                    detail=detail_builder(response),
                    context=request_context or self._request_context(request),
                )
            )
        except Exception as exc:
            self._results.append(
                CheckResult(
                    category="live",
                    name=result_name,
                    status="FAIL",
                    detail=f"Service call failed: {exc}",
                    context=request_context or self._request_context(request),
                )
            )

    def _parameter_list(
        self,
        name: str,
        *,
        expected_len: int,
        cast: Callable[[Any], Any],
    ) -> list[Any]:
        values = list(self.get_parameter(name).value)
        if len(values) != expected_len:
            raise ValueError(f"{name} must contain {expected_len} values")
        return [cast(value) for value in values]

    def _json_detail(self, **payload: Any) -> str:
        return json.dumps(payload, ensure_ascii=False)

    def _camera_optional_result(
        self,
        response: Any,
        *,
        result_name: str,
        detail_builder: Callable[[Any], str],
    ) -> CheckResult:
        success = bool(getattr(response, "success", True))
        detail = detail_builder(response)
        if success:
            return CheckResult(
                category="live",
                name=result_name,
                status="PASS",
                detail=detail,
            )

        message = str(getattr(response, "message", ""))
        if (not self._camera_hardware_available()) and (
            "failed to open camera" in message.lower()
        ):
            return CheckResult(
                category="live",
                name=result_name,
                status="SKIP",
                detail=(
                    "Skipped physical camera capture because no camera hardware is installed in the current environment. "
                    "The ROS2 service path itself is reachable and this run keeps only the logic/interface validation."
                ),
                recommendation=(
                    "Reconnect a camera and rerun with camera_hardware_available:=true when you need full visual input validation."
                ),
            )

        return CheckResult(
            category="live",
            name=result_name,
            status="FAIL",
            detail=detail,
            recommendation=(
                "Review the raw service message and request context, then fix the service-side implementation before rerunning."
            ),
        )

    def _joint_result(
        self,
        response: Any,
        *,
        result_name: str,
        detail_json: str,
    ) -> CheckResult:
        raw_angles = list(getattr(response, "target_joint_angles", []))
        try:
            normalized = [float(value) for value in raw_angles]
        except Exception:
            normalized = []
        if any(not math.isfinite(value) for value in normalized):
            return CheckResult(
                category="live",
                name=result_name,
                status="FAIL",
                detail=self._json_detail(
                    message="关节规划结果包含非有限值（NaN/Inf）",
                    target_joint_angles=raw_angles,
                    detail_json=detail_json,
                ),
                recommendation="先修复示教笛卡尔规划或 IK 选解问题，再重新执行该项验收。",
            )
        success = bool(getattr(response, "success", True))
        return CheckResult(
            category="live",
            name=result_name,
            status="PASS" if success else "FAIL",
            detail=self._json_detail(
                message=getattr(response, "message", ""),
                target_joint_angles=raw_angles,
                detail_json=detail_json,
            ),
            recommendation="" if success else "请检查示教规划返回结果和当前关节状态后重新验收。",
        )

    def _request_context(self, request: Any) -> str:
        payload = {}
        for field in getattr(request, "__slots__", []):
            name = field[1:] if field.startswith("_") else field
            value = getattr(request, field)
            payload[name] = self._to_jsonable(value)
        return json.dumps(payload, ensure_ascii=False)

    def _to_jsonable(self, value: Any) -> Any:
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        if isinstance(value, (list, tuple)):
            return [self._to_jsonable(item) for item in value]
        if hasattr(value, "tolist") and callable(value.tolist):
            try:
                return self._to_jsonable(value.tolist())
            except Exception:
                pass
        if hasattr(value, "typecode") and hasattr(value, "__iter__"):
            try:
                return [self._to_jsonable(item) for item in list(value)]
            except Exception:
                pass
        if hasattr(value, "__slots__"):
            nested = {}
            for field in getattr(value, "__slots__", []):
                name = field[1:] if field.startswith("_") else field
                nested[name] = self._to_jsonable(getattr(value, field))
            return nested
        return str(value)

    def _wait_for_future(self, future, *, timeout_sec: float):
        rclpy.spin_until_future_complete(
            self, future, timeout_sec=max(0.1, float(timeout_sec))
        )
        if not future.done():
            raise TimeoutError("ROS2 call timed out")
        return future.result()

    def _skip(
        self,
        category: str,
        name: str,
        detail: str,
        *,
        recommendation: str = "",
    ) -> None:
        self._results.append(
            CheckResult(
                category=category,
                name=name,
                status="SKIP",
                detail=detail,
                recommendation=recommendation,
            )
        )

    def _format_status_detail(self, message: ArmStatus) -> str:
        return (
            f"hardware_connected={bool(message.hardware_connected)}, "
            f"motors_enabled={bool(message.motors_enabled)}, "
            f"warnings={len(message.warnings)}"
        )

    def _format_joint_state_detail(self, message: JointState) -> str:
        return (
            f"joint_count={len(message.name)}, "
            f"positions={len(message.position)}"
        )

    def _on_status(self, message: ArmStatus) -> None:
        self._status_msg = message

    def _on_joint_state(self, message: JointState) -> None:
        self._joint_state_msg = message

    def _counts(self) -> dict[str, int]:
        counts = {"PASS": 0, "WARN": 0, "FAIL": 0, "SKIP": 0}
        for result in self._results:
            counts[result.status] = counts.get(result.status, 0) + 1
        return counts

    def _overall_status(self) -> str:
        counts = self._counts()
        if counts["FAIL"] > 0:
            return "FAIL"
        if counts["WARN"] > 0:
            return "WARN"
        if counts["PASS"] > 0:
            return "PASS"
        return "SKIP"

    def _overall_conclusion(self) -> str:
        counts = self._counts()
        if counts["FAIL"] > 0:
            return "当前环境仍有失败项，建议先处理 FAIL 项。"
        if counts["WARN"] > 0:
            return "当前环境主链路已通过，但仍有提示项需要关注。"
        return "当前环境主链路检查通过。"

    def _write_reports(self) -> None:
        report_dir_value = str(self.get_parameter("report_dir").value).strip()
        report_dir = Path(report_dir_value) if report_dir_value else Path.cwd()
        report_dir.mkdir(parents=True, exist_ok=True)
        json_path = report_dir / "horizon_arm_system_check_results.json"
        html_path = report_dir / "horizon_arm_system_check_results.html"

        self._results.append(
            CheckResult(
                category="report",
                name="System check results",
                status="PASS",
                detail=f"JSON: {json_path} | HTML: {html_path}",
            )
        )

        generated_at = dt.datetime.now()
        payload = {
            "generated_at_local": generated_at.isoformat(timespec="seconds"),
            "overall_status": self._overall_status(),
            "overall_conclusion": self._overall_conclusion(),
            "summary": self._counts(),
            "results": [self._serialize_result(item) for item in self._results],
            "acceptance_profile": str(self.get_parameter("acceptance_profile").value),
            "allow_hardware_side_effects": bool(
                self.get_parameter("allow_hardware_side_effects").value
            ),
            "test_matrix": self._build_test_matrix(),
        }
        payload["category_summary"] = self._build_category_summary(payload["results"])
        payload["new_interface_coverage"] = self._build_new_interface_coverage(
            payload["results"]
        )
        payload["action_items"] = self._build_action_items(payload["results"])

        json_path.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        html_path.write_text(self._render_html(payload), encoding="utf-8")

    def _serialize_result(self, result: CheckResult) -> dict[str, Any]:
        meta = self._result_meta(result)
        return {
            "category": result.category,
            "category_label": self._category_label(result.category),
            "name": result.name,
            "display_name": meta["display_name"],
            "actual_name": meta["actual_name"],
            "status": result.status,
            "status_label": self._status_label(result.status),
            "detail": result.detail,
            "detail_zh": self._detail_zh(result),
            "recommendation": result.recommendation,
            "recommendation_zh": self._recommendation_zh(result),
            "interpretation": self._interpretation(result),
        }

    def _result_meta(self, result: CheckResult) -> dict[str, str]:
        mapping = {
            "/horizon_arm/status": ("硬件状态话题", "/horizon_arm/status"),
            "/horizon_arm/joint_states": ("关节状态话题", "/horizon_arm/joint_states"),
            "/horizon_arm/enable": ("机械臂使能服务", "/horizon_arm/enable"),
            "/horizon_arm/disable": ("机械臂失能服务", "/horizon_arm/disable"),
            "/horizon_arm/emergency_stop": ("机械臂急停服务", "/horizon_arm/emergency_stop"),
            "/horizon_arm/set_digital_output": (
                "数字输出服务",
                "/horizon_arm/set_digital_output",
            ),
            "/horizon_arm/set_gripper_state": (
                "夹爪开合服务",
                "/horizon_arm/set_gripper_state",
            ),
            "/horizon_arm/visual_grasp": ("视觉抓取服务", "/horizon_arm/visual_grasp"),
            "/horizon_arm/follow_grasp_control": (
                "视觉跟随抓取控制服务",
                "/horizon_arm/follow_grasp_control",
            ),
            "/horizon_arm/joycon_control": (
                "Joycon 控制服务",
                "/horizon_arm/joycon_control",
            ),
            "/horizon_arm/embodied_instruction": (
                "具身智能指令服务",
                "/horizon_arm/embodied_instruction",
            ),
            "/horizon_arm_controller/follow_joint_trajectory": (
                "关节轨迹动作接口",
                "/horizon_arm_controller/follow_joint_trajectory",
            ),
            "/horizon_arm/run_instruction": (
                "统一指令动作接口",
                "/horizon_arm/run_instruction",
            ),
            "Embodied_SDK": ("Linux SDK 主模块", "Embodied_SDK"),
            "MotionSDK": ("运动 SDK", "MotionSDK"),
            "VisualGraspSDK": ("视觉抓取 SDK", "VisualGraspSDK"),
            "FollowGraspSDK": ("视觉跟随抓取 SDK", "FollowGraspSDK"),
            "JoyconSDK": ("Joycon SDK", "JoyconSDK"),
            "IOSDK": ("IO SDK", "IOSDK"),
            "ZDTGripperSDK": ("夹爪 SDK", "ZDTGripperSDK"),
            "HorizonArmSDK": ("机械臂聚合 SDK", "HorizonArmSDK"),
            "AISDK": ("AI SDK", "AISDK"),
            "DepthEstimationSDK": ("深度估计 SDK", "DepthEstimationSDK"),
            "EmbodiedSDK": ("具身智能 SDK", "EmbodiedSDK"),
            "EmbodiedSDK live init": ("具身智能 SDK 实例化", "EmbodiedSDK"),
            "HorizonArmSDK live init": ("机械臂聚合 SDK 实例化", "HorizonArmSDK"),
            "Gripper ROS2 wrapper": ("夹爪 ROS2 封装", "Gripper ROS2 wrapper"),
            "Vision/Follow ROS2 wrapper": (
                "视觉抓取 / 跟随 ROS2 封装",
                "Vision/Follow ROS2 wrapper",
            ),
            "Joycon ROS2 wrapper": ("Joycon ROS2 封装", "Joycon ROS2 wrapper"),
            "Embodied AI ROS2 wrapper": (
                "具身智能 ROS2 封装",
                "Embodied AI ROS2 wrapper",
            ),
            "RunInstruction smoke test": ("统一指令烟测", "RunInstruction smoke test"),
            "Preset motion test": ("预设动作测试", "Preset motion test"),
            "Single-axis motion test": ("单轴运动测试", "Single-axis motion test"),
            "Multi-axis motion test": ("多轴运动测试", "Multi-axis motion test"),
            "Digital output live test": ("数字输出实测", "Digital output live test"),
            "Gripper close test": ("夹爪闭合测试", "Gripper close test"),
            "Gripper open test": ("夹爪张开测试", "Gripper open test"),
            "Visual grasp health test": (
                "视觉抓取健康检查",
                "Visual grasp health test",
            ),
            "Follow grasp health test": (
                "跟随抓取健康检查",
                "Follow grasp health test",
            ),
            "Joycon status test": ("Joycon 状态检查", "Joycon status test"),
            "Embodied health test": ("具身智能健康检查", "Embodied health test"),
            "System check results": ("自检结果文件", "System check results"),
        }
        if result.name.startswith("Preset motion test ("):
            return {"display_name": "预设动作测试", "actual_name": result.name}
        display_name, actual_name = mapping.get(result.name, (result.name, result.name))
        return {"display_name": display_name, "actual_name": actual_name}

    def _category_label(self, category: str) -> str:
        mapping = {
            "ros_runtime": "ROS2 运行时",
            "sdk": "Linux SDK",
            "capability": "能力封装",
            "live": "实机/实链路测试",
            "report": "结果文件",
        }
        return mapping.get(category, category)

    def _status_label(self, status: str) -> str:
        mapping = {
            "PASS": "通过",
            "WARN": "提示",
            "FAIL": "失败",
            "SKIP": "跳过",
        }
        return mapping.get(status, status)

    def _status_color(self, status: str) -> str:
        mapping = {
            "PASS": "#15803d",
            "WARN": "#b45309",
            "FAIL": "#b42318",
            "SKIP": "#475467",
        }
        return mapping.get(status, "#333333")

    def _detail_zh(self, result: CheckResult) -> str:
        detail = result.detail
        replacements = [
            ("hardware_connected=True", "硬件已连接"),
            ("hardware_connected=False", "硬件未连接"),
            ("motors_enabled=True", "电机已使能"),
            ("motors_enabled=False", "电机未使能"),
            ("warnings=", "告警数量="),
            ("joint_count=", "关节数量="),
            ("positions=", "位置数量="),
            ("Module import succeeded.", "模块导入成功。"),
            ("Attribute import succeeded.", "属性导入成功。"),
            ("Instantiation succeeded:", "实例化成功:"),
            ("Service became ready within", "服务在以下时间内就绪:"),
            ("Service was not ready within", "服务在以下时间内未就绪:"),
            ("Action server became ready within", "动作接口在以下时间内就绪:"),
            ("Action server was not ready within", "动作接口在以下时间内未就绪:"),
            ("RunInstruction action server is not ready.", "统一指令动作接口未就绪。"),
            ("Goal was rejected.", "动作目标被拒绝。"),
            ("Digital output service is not ready.", "数字输出服务未就绪。"),
            ("Gripper service is not ready.", "夹爪服务未就绪。"),
            ("Visual grasp service is not ready.", "视觉抓取服务未就绪。"),
            ("Follow grasp service is not ready.", "视觉跟随抓取服务未就绪。"),
            ("Joycon control service is not ready.", "Joycon 控制服务未就绪。"),
            ("Embodied instruction service is not ready.", "具身智能指令服务未就绪。"),
            ("Disabled by parameter.", "当前由参数关闭，未执行。"),
            ("No message received within", "在以下时间内未收到消息:"),
            ("Import failed:", "导入失败:"),
            ("Check failed:", "检查失败:"),
            ("Execution failed:", "执行失败:"),
            ("Service call failed:", "服务调用失败:"),
            ("ROS2 call timed out", "ROS2 调用超时"),
            ("visual grasp wrapper health check passed", "视觉抓取 wrapper 健康检查通过"),
            ("follow grasp wrapper health check passed", "跟随抓取 wrapper 健康检查通过"),
            ("follow grasp wrapper status queried", "跟随抓取 wrapper 状态查询成功"),
            ("joycon wrapper status queried", "Joycon wrapper 状态查询成功"),
            ("embodied wrapper health check passed", "具身智能 wrapper 健康检查通过"),
        ]
        zh = detail
        for src, dst in replacements:
            zh = zh.replace(src, dst)
        return zh

    def _recommendation_zh(self, result: CheckResult) -> str:
        recommendation = result.recommendation
        mapping = {
            "Enable instantiate_embodied_sdk:=true only in a fully configured AI environment.": "仅在 AI 凭据、模型配置、网络环境都准备完整后，再开启 instantiate_embodied_sdk:=true。",
            "Enable instantiate_horizon_sdk:=true after preparing a controlled test harness.": "请在准备好受控测试环境后，再开启 instantiate_horizon_sdk:=true。",
            "Use the gripper ROS2 service or RunInstruction for quick Linux verification.": "日常自检时，优先使用夹爪服务或统一指令接口确认链路正常。",
            "Use the wrapper services for health checks first, then enable real camera tasks during integration.": "先做 wrapper 健康检查，再在接入相机和真实场景后做视觉任务联调。",
            "Use the status command by default and connect/start only when the controller is present.": "默认优先使用 status 检查；只有手柄硬件到位时再做 connect/start。",
            "Use the health command by default and enable real NL control only after AI credentials are configured.": "默认优先使用 health 检查；真实自然语言控制请在 AI 凭据配置完成后再启用。",
        }
        if recommendation:
            return mapping.get(recommendation, recommendation)
        if result.status == "PASS":
            return "当前检查项已通过。"
        if result.status == "SKIP":
            return "如需覆盖该项，请根据场景打开对应参数后重新执行自检。"
        return ""

    def _interpretation(self, result: CheckResult) -> str:
        if result.status == "PASS":
            if result.category == "ros_runtime":
                return "这说明对应的 ROS2 话题、服务或动作接口已经正常提供。"
            if result.category == "sdk":
                return "这说明 Linux SDK 的对应模块已经能在当前环境中被导入或实例化。"
            if result.category == "live":
                return "这说明该实机/实链路动作在当前环境下已经跑通。"
            return "这说明当前能力已经具备并可正常使用。"
        if result.status == "WARN":
            return "这不是直接失败，但仍建议注意使用前提和限制。"
        if result.status == "FAIL":
            return "这会影响实际使用，建议先处理。"
        return "当前默认未执行该检查项，需要在合适的硬件或配置条件下手动开启。"

    def _render_html(self, payload: dict[str, Any]) -> str:
        summary = payload["summary"]
        table_rows = "\n".join(self._render_result_row(item) for item in payload["results"])
        return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Horizon Arm ROS2 自检结果</title>
  <style>
    :root {{
      --bg: #f3f6fb;
      --panel: #ffffff;
      --text: #17212f;
      --muted: #5b6678;
      --line: #d8e0ec;
      --pass: #15803d;
      --warn: #b45309;
      --fail: #b42318;
      --skip: #475467;
      --accent: #0f766e;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Microsoft YaHei", "PingFang SC", "Noto Sans SC", sans-serif;
      color: var(--text);
      background:
        radial-gradient(circle at top right, #d8fff5 0, rgba(216,255,245,0) 28%),
        linear-gradient(180deg, #eff5ff 0%, var(--bg) 34%, #eef2f8 100%);
    }}
    .wrap {{
      max-width: 1400px;
      margin: 0 auto;
      padding: 24px 18px 40px;
    }}
    .hero {{
      background: linear-gradient(135deg, #0f172a 0%, #0f766e 55%, #14b8a6 100%);
      color: #fff;
      border-radius: 20px;
      padding: 22px 24px;
      box-shadow: 0 24px 60px rgba(15, 23, 42, 0.22);
    }}
    .hero h1 {{
      margin: 0 0 8px;
      font-size: 28px;
    }}
    .hero p {{
      margin: 4px 0 0;
      line-height: 1.6;
      color: rgba(255,255,255,0.92);
    }}
    .summary {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(150px, 1fr));
      gap: 12px;
      margin: 18px 0;
    }}
    .tile, .panel, .table-wrap {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 16px;
      box-shadow: 0 10px 24px rgba(15, 23, 42, 0.05);
    }}
    .tile {{
      padding: 16px 18px;
    }}
    .tile .label {{
      color: var(--muted);
      font-size: 13px;
      margin-bottom: 6px;
    }}
    .tile .value {{
      font-size: 28px;
      font-weight: 700;
    }}
    .panels {{
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      margin-bottom: 16px;
    }}
    .panel {{
      padding: 16px 18px;
    }}
    .panel h2 {{
      margin: 0 0 10px;
      font-size: 16px;
    }}
    .panel p, .panel li {{
      line-height: 1.65;
      font-size: 14px;
      color: var(--muted);
    }}
    .panel ul {{
      margin: 0;
      padding-left: 18px;
    }}
    .table-wrap {{
      overflow: hidden;
    }}
    table {{
      width: 100%;
      border-collapse: collapse;
      table-layout: fixed;
    }}
    thead th {{
      text-align: left;
      padding: 12px 10px;
      background: #eef4fb;
      border-bottom: 1px solid var(--line);
      font-size: 13px;
      color: #334155;
    }}
    tbody td {{
      vertical-align: top;
      padding: 12px 10px;
      border-bottom: 1px solid #e7edf5;
      font-size: 13px;
      line-height: 1.6;
    }}
    tbody tr:hover {{
      background: #f8fbff;
    }}
    .badge {{
      display: inline-flex;
      align-items: center;
      border-radius: 999px;
      padding: 3px 10px;
      font-size: 12px;
      font-weight: 700;
      color: #fff;
    }}
    .badge.pass {{ background: var(--pass); }}
    .badge.warn {{ background: var(--warn); }}
    .badge.fail {{ background: var(--fail); }}
    .badge.skip {{ background: var(--skip); }}
    .mono {{
      font-family: "Cascadia Mono", "Consolas", monospace;
      word-break: break-word;
      color: #0f172a;
      background: #f8fafc;
      border: 1px solid #e5eaf2;
      border-radius: 8px;
      padding: 8px;
    }}
    .muted {{
      color: var(--muted);
    }}
    @media (max-width: 900px) {{
      .panels {{ grid-template-columns: 1fr; }}
      .hero h1 {{ font-size: 24px; }}
      table, thead, tbody, th, td, tr {{
        display: block;
      }}
      thead {{
        display: none;
      }}
      tbody td {{
        border-bottom: none;
        padding-top: 6px;
        padding-bottom: 6px;
      }}
      tbody tr {{
        display: block;
        padding: 10px;
        border-bottom: 1px solid #e7edf5;
      }}
    }}
  </style>
</head>
<body>
  <div class="wrap">
    <section class="hero">
      <h1>Horizon Arm ROS2 自检结果</h1>
      <p>{html.escape(payload["overall_conclusion"])}</p>
      <p>生成时间：{html.escape(payload["generated_at_local"])}</p>
    </section>

    <section class="summary">
      <div class="tile"><div class="label">总体状态</div><div class="value">{html.escape(self._status_label(payload["overall_status"]))}</div></div>
      <div class="tile"><div class="label">通过 PASS</div><div class="value">{summary["PASS"]}</div></div>
      <div class="tile"><div class="label">提示 WARN</div><div class="value">{summary["WARN"]}</div></div>
      <div class="tile"><div class="label">失败 FAIL</div><div class="value">{summary["FAIL"]}</div></div>
      <div class="tile"><div class="label">跳过 SKIP</div><div class="value">{summary["SKIP"]}</div></div>
    </section>

    <section class="panels">
      <div class="panel">
        <h2>怎么看这份报告</h2>
        <ul>
          <li><strong>通过</strong>：该项已在当前环境跑通。</li>
          <li><strong>提示</strong>：不是失败，但仍建议注意前提条件和限制。</li>
          <li><strong>失败</strong>：当前功能或环境还有问题，建议先处理。</li>
          <li><strong>跳过</strong>：默认未执行，通常因为需要额外硬件或配置。</li>
        </ul>
      </div>
      <div class="panel">
        <h2>怎么看实际参数</h2>
        <p>本页同时保留中文结果说明和原始结果字符串。原始结果里会直接显示真实话题名、服务名、动作名、SDK 类名、返回 message 和关键参数，方便开发者排障。</p>
      </div>
    </section>

    <section class="table-wrap">
      <table>
        <thead>
          <tr>
            <th style="width: 90px;">状态</th>
            <th style="width: 120px;">分类</th>
            <th style="width: 180px;">中文名称</th>
            <th style="width: 240px;">实际名称</th>
            <th>原始结果 / 实际参数</th>
            <th>中文说明</th>
            <th style="width: 240px;">建议</th>
          </tr>
        </thead>
        <tbody>
          {table_rows}
        </tbody>
      </table>
    </section>
  </div>
</body>
</html>
"""

    def _render_result_row(self, item: dict[str, Any]) -> str:
        status_class = item["status"].lower()
        recommendation = html.escape(item["recommendation_zh"] or "-")
        raw_detail = html.escape(item["detail"])
        zh_detail = html.escape(item["detail_zh"])
        interpretation = html.escape(item["interpretation"])
        return f"""
<tr>
  <td><span class="badge {status_class}">{html.escape(item["status_label"])}</span></td>
  <td>{html.escape(item["category_label"])}</td>
  <td>{html.escape(item["display_name"])}</td>
  <td><div class="mono">{html.escape(item["actual_name"])}</div></td>
  <td><div class="mono">{raw_detail}</div></td>
  <td>
    <div>{zh_detail}</div>
    <div class="muted" style="margin-top: 6px;">{interpretation}</div>
  </td>
  <td>{recommendation}</td>
</tr>
"""

    def _result_meta(self, result: CheckResult) -> dict[str, str]:
        mapping = {
            "/horizon_arm/status": ("硬件状态话题", "/horizon_arm/status"),
            "/horizon_arm/joint_states": ("关节状态话题", "/horizon_arm/joint_states"),
            "/horizon_arm/enable": ("机械臂使能服务", "/horizon_arm/enable"),
            "/horizon_arm/disable": ("机械臂失能服务", "/horizon_arm/disable"),
            "/horizon_arm/emergency_stop": ("机械臂急停服务", "/horizon_arm/emergency_stop"),
            "/horizon_arm/set_digital_output": ("数字输出服务", "/horizon_arm/set_digital_output"),
            "/horizon_arm/set_gripper_state": ("夹爪控制服务", "/horizon_arm/set_gripper_state"),
            "/horizon_arm/visual_grasp": ("视觉抓取服务", "/horizon_arm/visual_grasp"),
            "/horizon_arm/visual_grasp_ex": ("增强视觉抓取服务", "/horizon_arm/visual_grasp_ex"),
            "/horizon_arm/vision_config": ("视觉配置服务", "/horizon_arm/vision_config"),
            "/horizon_arm/pick_hsv": ("HSV取样服务", "/horizon_arm/pick_hsv"),
            "/horizon_arm/detect_target": ("目标检测服务", "/horizon_arm/detect_target"),
            "/horizon_arm/follow_grasp_control": ("跟随抓取控制服务", "/horizon_arm/follow_grasp_control"),
            "/horizon_arm/follow_target": ("目标跟随服务", "/horizon_arm/follow_target"),
            "/horizon_arm/joycon_control": ("Joycon控制服务", "/horizon_arm/joycon_control"),
            "/horizon_arm/joycon_advanced_control": ("Joycon高级控制服务", "/horizon_arm/joycon_advanced_control"),
            "/horizon_arm/teach_jog": ("示教点动服务", "/horizon_arm/teach_jog"),
            "/horizon_arm/embodied_instruction": ("具身智能指令服务", "/horizon_arm/embodied_instruction"),
            "/horizon_arm/embodied_command": ("具身智能命令服务", "/horizon_arm/embodied_command"),
            "/horizon_arm_controller/follow_joint_trajectory": ("关节轨迹动作接口", "/horizon_arm_controller/follow_joint_trajectory"),
            "/horizon_arm/run_instruction": ("统一指令动作接口", "/horizon_arm/run_instruction"),
            "/horizon_arm/teaching_program": ("示教程序动作接口", "/horizon_arm/teaching_program"),
            "Gripper ROS2 wrapper": ("夹爪ROS2封装", "Gripper ROS2 wrapper"),
            "Vision/Follow ROS2 wrapper": ("视觉/跟随ROS2封装", "Vision/Follow ROS2 wrapper"),
            "Joycon ROS2 wrapper": ("Joycon ROS2封装", "Joycon ROS2 wrapper"),
            "Teaching ROS2 wrapper": ("示教ROS2封装", "Teaching ROS2 wrapper"),
            "Embodied AI ROS2 wrapper": ("具身智能ROS2封装", "Embodied AI ROS2 wrapper"),
            "RunInstruction smoke test": ("统一指令烟测", "RunInstruction smoke test"),
            "Preset motion test": ("预设动作测试", "Preset motion test"),
            "Single-axis motion test": ("单轴运动测试", "Single-axis motion test"),
            "Multi-axis motion test": ("多轴运动测试", "Multi-axis motion test"),
            "Digital output live test": ("数字输出实测", "Digital output live test"),
            "Gripper close test": ("夹爪闭合测试", "Gripper close test"),
            "Gripper open test": ("夹爪张开测试", "Gripper open test"),
            "Visual grasp health test": ("视觉抓取健康检查", "Visual grasp health test"),
            "Follow grasp health test": ("跟随抓取健康检查", "Follow grasp health test"),
            "Joycon status test": ("Joycon状态检查", "Joycon status test"),
            "Embodied health test": ("具身智能健康检查", "Embodied health test"),
            "Vision config update test": ("视觉配置更新测试", "Vision config update test"),
            "Pick HSV sample test": ("HSV取样测试", "Pick HSV sample test"),
            "Detect target test": ("目标检测测试", "Detect target test"),
            "Visual grasp ex dry-run test": ("增强视觉抓取DryRun测试", "Visual grasp ex dry-run test"),
            "Follow target status test": ("目标跟随状态测试", "Follow target status test"),
            "Follow target manual target test": ("目标跟随手动设框测试", "Follow target manual target test"),
            "Teach jog joint dry-run test": ("示教关节点动DryRun测试", "Teach jog joint dry-run test"),
            "Teach jog cartesian dry-run test": ("示教笛卡尔DryRun测试", "Teach jog cartesian dry-run test"),
            "Teaching program validate test": ("示教程序校验测试", "Teaching program validate test"),
            "Embodied functions test": ("具身函数列表测试", "Embodied functions test"),
            "Embodied actions test": ("具身动作列表测试", "Embodied actions test"),
            "System check results": ("报告文件输出", "System check results"),
        }
        if result.name.startswith("Preset motion test ("):
            return {"display_name": "预设动作测试", "actual_name": result.name}
        display_name, actual_name = mapping.get(result.name, (result.name, result.name))
        return {"display_name": display_name, "actual_name": actual_name}

    def _category_label(self, category: str) -> str:
        mapping = {
            "ros_runtime": "ROS2运行时",
            "sdk": "Linux SDK",
            "capability": "功能封装",
            "live": "实测验收",
            "report": "报告输出",
        }
        return mapping.get(category, category)

    def _status_label(self, status: str) -> str:
        mapping = {
            "PASS": "通过",
            "WARN": "提示",
            "FAIL": "失败",
            "SKIP": "跳过",
        }
        return mapping.get(status, status)

    def _detail_zh(self, result: CheckResult) -> str:
        detail = str(result.detail)
        if detail.startswith("{") and detail.endswith("}"):
            return detail
        if result.status == "SKIP":
            return "该项未执行，通常是参数未开启，或依赖额外硬件/环境。"
        return detail

    def _recommendation_zh(self, result: CheckResult) -> str:
        if result.recommendation:
            return result.recommendation
        if result.status == "FAIL":
            return "先查看原始结果和上下文参数，确认服务或动作接口状态，再修复后重新验收。"
        if result.status == "SKIP":
            return "仅在相机、控制器、AI 配置或机械臂现场条件准备好后，再开启该项。"
        return "当前无需额外处理。"

    def _interpretation(self, result: CheckResult) -> str:
        if result.status == "PASS":
            if result.category == "live":
                return "该运行链路已在当前环境下实际跑通。"
            if result.category == "ros_runtime":
                return "该 ROS2 话题、服务或动作接口已正常可见。"
            if result.category == "sdk":
                return "该 SDK 模块或类在当前环境下可正常导入。"
            return "该能力封装已暴露，可继续集成联调。"
        if result.status == "FAIL":
            return "该项当前仍是阻塞问题，或会降低整体验收可信度。"
        if result.status == "WARN":
            return "该项当前不是失败，但在更大范围部署前仍需关注。"
        return "该项本次没有实际执行。"

    def _context_zh(self, result: dict[str, Any]) -> str:
        raw = str(result.get("context", "") or "").strip()
        if not raw:
            category = str(result.get("category", ""))
            if category == "ros_runtime":
                return "无请求参数（运行时可见性检查）"
            if category == "sdk":
                return "无请求参数（SDK导入/实例化检查）"
            if category == "capability":
                return "无请求参数（静态能力暴露检查）"
            if category == "report":
                return "无请求参数（结果文件生成检查）"
            return "无请求参数"
        try:
            payload = json.loads(raw)
        except Exception:
            return raw

        key_map = {
            "command": "命令",
            "mode": "模式",
            "pipeline": "视觉管线",
            "target_class": "目标类别",
            "conf_thres": "置信度阈值",
            "iou_thres": "IOU阈值",
            "interval_sec": "执行间隔(秒)",
            "use_hsv": "启用HSV",
            "use_depth": "启用深度",
            "depth_min_m": "最小深度(米)",
            "depth_max_m": "最大深度(米)",
            "u": "像素U",
            "v": "像素V",
            "x1": "框选左上X",
            "y1": "框选左上Y",
            "x2": "框选右下X",
            "y2": "框选右下Y",
            "dry_run": "仅规划不执行",
            "use_click": "使用点击点",
            "use_bbox": "使用框选",
            "z_offset_m": "Z偏移(米)",
            "approach_height_m": "接近高度(米)",
            "grasp_depth_m": "抓取深度(米)",
            "pre_grasp_rpy": "预抓取姿态RPY",
            "options_json": "扩展参数",
            "joint_index": "关节序号",
            "delta": "位移/角度增量",
            "frame": "参考坐标系",
            "axis": "操作轴",
            "interpolation_type": "插补类型",
            "linear_velocity": "线速度",
            "angular_velocity": "角速度",
            "follow_distance_m": "跟随距离(米)",
            "deadband_px": "死区像素",
            "max_linear_speed": "最大线速度",
            "max_angular_speed": "最大角速度",
            "auto_grasp": "自动抓取",
            "channel": "IO通道",
            "state": "目标状态",
            "stream": "流式模式",
            "provider": "AI厂商",
            "model": "AI模型",
            "control_mode": "控制模式",
            "instruction": "自然语言指令",
            "joint_angles": "目标关节角",
            "position": "目标位置",
            "orientation": "目标姿态",
            "linear_acceleration": "线加速度",
            "angular_acceleration": "角加速度",
            "joint_max_velocities": "关节最大速度",
            "joint_max_accelerations": "关节最大加速度",
        }
        lines = []
        for key, value in payload.items():
            label = key_map.get(key, key)
            if isinstance(value, bool):
                value_text = "是" if value else "否"
            else:
                value_text = json.dumps(value, ensure_ascii=False) if isinstance(value, (dict, list)) else str(value)
            lines.append(f"{label}: {value_text}")
        return "\n".join(lines) if lines else "-"

    def _reference_hint(self, result: dict[str, Any]) -> str:
        status = str(result.get("status", ""))
        name = str(result.get("name", ""))
        if name == "/horizon_arm/status":
            return "正常参考：hardware_connected=True，motors_enabled=True，warnings=0。"
        if name == "/horizon_arm/joint_states":
            return "正常参考：6个关节名称和6个位置值均存在。"
        if name.endswith("/set_digital_output"):
            return "正常参考：有IO硬件时服务应就绪；当前无IO环境时允许跳过或仅做接口验证。"
        if "Vision config update test" == name:
            return "正常参考：返回 vision config updated，并能看到配置回显。"
        if "Visual grasp ex dry-run test" == name:
            return "正常参考：dry_run 应返回 accepted，不应报序列化或相机链路错误。"
        if "Follow target status test" == name:
            return "正常参考：能返回 running/state_json，说明状态接口链路正常。"
        if "Teach jog joint dry-run test" == name:
            return "正常参考：应返回6个有限数值的目标关节角，不应出现 NaN/Inf。"
        if "Teach jog cartesian dry-run test" == name:
            return "正常参考：应返回6个有限数值的目标关节角；出现 NaN/Inf 说明 IK 或选解异常。"
        if "Teaching program validate test" == name:
            return "正常参考：返回 program validated，并包含 points 校验结果。"
        if "Embodied health test" == name or "Embodied functions test" == name or "Embodied actions test" == name:
            return "正常参考：应成功返回健康结果、函数列表或动作列表，不应出现缺模块异常。"
        if status == "PASS":
            return "正常参考：该项当前返回结果已满足验收预期。"
        if status == "SKIP":
            return "说明：该项本次未执行，不能作为功能已完成的证明。"
        return "说明：结合原始结果、请求参数和服务返回内容一起判断问题。"

    def _build_category_summary(self, results: list[dict[str, Any]]) -> list[dict[str, Any]]:
        buckets: dict[str, dict[str, Any]] = {}
        for item in results:
            key = str(item["category"])
            bucket = buckets.setdefault(
                key,
                {
                    "category": key,
                    "category_label": item["category_label"],
                    "PASS": 0,
                    "WARN": 0,
                    "FAIL": 0,
                    "SKIP": 0,
                    "total": 0,
                },
            )
            bucket[item["status"]] = bucket.get(item["status"], 0) + 1
            bucket["total"] += 1
        return list(buckets.values())

    def _build_new_interface_coverage(self, results: list[dict[str, Any]]) -> list[dict[str, Any]]:
        tracked = [
            ("Vision config update test", "VisionConfig"),
            ("Pick HSV sample test", "PickHSV"),
            ("Detect target test", "DetectTarget"),
            ("Visual grasp ex dry-run test", "VisualGraspEx"),
            ("Follow target status test", "FollowTarget status"),
            ("Follow target manual target test", "FollowTarget set_target"),
            ("Teach jog joint dry-run test", "TeachJog joint_jog"),
            ("Teach jog cartesian dry-run test", "TeachJog cartesian"),
            ("Teaching program validate test", "TeachingProgram validate"),
            ("Embodied functions test", "EmbodiedCommand functions"),
            ("Embodied actions test", "EmbodiedCommand actions"),
        ]
        result_map = {str(item["name"]): item for item in results}
        coverage = []
        for key, label in tracked:
            item = result_map.get(key)
            if item is None:
                coverage.append(
                    {
                        "label": label,
                        "status": "MISSING",
                        "status_label": "缺失",
                        "detail": "该接口本次没有生成结果。",
                    }
                )
                continue
            coverage.append(
                {
                    "label": label,
                    "status": item["status"],
                    "status_label": item["status_label"],
                    "detail": item["detail"],
                }
            )
        return coverage

    def _build_action_items(self, results: list[dict[str, Any]]) -> list[dict[str, str]]:
        items = []
        for item in results:
            if item["status"] == "FAIL":
                items.append(
                    {
                        "name": item["display_name"],
                        "status": item["status"],
                        "recommendation": item["recommendation_zh"],
                    }
                )
                continue
            if item["status"] == "SKIP" and item["name"] == "Digital output live test":
                continue
            if item["status"] == "SKIP":
                items.append(
                    {
                        "name": item["display_name"],
                        "status": item["status"],
                        "recommendation": item["recommendation_zh"],
                    }
                )
        return items[:12]

    def _build_test_matrix(self) -> list[dict[str, Any]]:
        matrix = [
            ("RunInstruction smoke test", "instruction_smoke_test", True, "motion"),
            ("Preset motion test", "preset_test_enabled", True, "motion"),
            ("Single-axis motion test", "single_axis_motion_test_enabled", True, "motion"),
            ("Multi-axis motion test", "multi_axis_motion_test_enabled", True, "motion"),
            ("Digital output live test", "digital_output_test_enabled", True, "io"),
            ("Gripper close/open test", "gripper_test_enabled", True, "gripper"),
            ("Visual grasp health test", "visual_grasp_health_test_enabled", False, "vision"),
            ("Follow grasp health test", "follow_grasp_health_test_enabled", False, "follow"),
            ("Joycon status test", "joycon_status_test_enabled", False, "joycon"),
            ("Embodied health test", "embodied_health_test_enabled", False, "ai"),
            ("Vision config update test", "vision_config_test_enabled", False, "vision"),
            ("Pick HSV sample test", "pick_hsv_test_enabled", False, "vision"),
            ("Detect target test", "detect_target_test_enabled", False, "vision"),
            ("Visual grasp ex dry-run test", "visual_grasp_ex_test_enabled", False, "vision"),
            ("Follow target status test", "follow_target_status_test_enabled", False, "follow"),
            ("Follow target manual target test", "follow_target_set_target_test_enabled", False, "follow"),
            ("Teach jog joint dry-run test", "teach_jog_joint_test_enabled", False, "teaching"),
            ("Teach jog cartesian dry-run test", "teach_jog_cartesian_test_enabled", False, "teaching"),
            ("Teaching program validate test", "teaching_program_validate_test_enabled", False, "teaching"),
            ("Embodied functions test", "embodied_functions_test_enabled", False, "ai"),
            ("Embodied actions test", "embodied_actions_test_enabled", False, "ai"),
        ]
        payload = []
        for label, param_name, side_effect, module in matrix:
            enabled = bool(self.get_parameter(param_name).value)
            required = True
            reason = "enabled in this run" if enabled else "disabled by parameter/profile"
            if side_effect and (not enabled) and (not bool(self.get_parameter("allow_hardware_side_effects").value)):
                reason = "not performed because this run did not enable real hardware testing"
            if label == "Digital output live test" and not self._io_hardware_available():
                required = False
                if enabled:
                    reason = "enabled in this run"
                else:
                    reason = (
                        "exempt in this run because no external IO hardware is installed; "
                        "logical interface coverage still comes from the ROS2 service readiness check"
                    )
            payload.append(
                {
                    "label": label,
                    "parameter": param_name,
                    "enabled": enabled,
                    "required": required,
                    "side_effect": side_effect,
                    "module": module,
                    "reason": reason,
                }
            )
        return payload

    def _render_html(self, payload: dict[str, Any]) -> str:
        summary = payload["summary"]
        category_cards = "\n".join(
            self._render_category_card(item) for item in payload["category_summary"]
        )
        coverage_cards = "\n".join(
            self._render_coverage_card(item)
            for item in payload["new_interface_coverage"]
        )
        action_items = "\n".join(
            f"<li><strong>{html.escape(item['name'])}</strong> [{html.escape(item['status'])}] {html.escape(item['recommendation'])}</li>"
            for item in payload["action_items"]
        ) or "<li>No follow-up items from this run.</li>"
        table_rows = "\n".join(self._render_result_row(item) for item in payload["results"])
        return f"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Horizon Arm ROS2 Validation Report</title>
  <style>
    :root {{
      --bg: #eef3f8;
      --panel: #ffffff;
      --text: #132033;
      --muted: #5d6b7c;
      --line: #d9e2ed;
      --pass: #166534;
      --warn: #b45309;
      --fail: #b91c1c;
      --skip: #475569;
      --shadow: 0 16px 40px rgba(15, 23, 42, 0.08);
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
      color: var(--text);
      background:
        radial-gradient(circle at top left, rgba(20,184,166,0.16), transparent 28%),
        linear-gradient(180deg, #f7fbff 0%, var(--bg) 100%);
    }}
    .wrap {{ max-width: 1480px; margin: 0 auto; padding: 24px 18px 40px; }}
    .hero {{
      background: linear-gradient(135deg, #0f172a 0%, #155e75 52%, #0f766e 100%);
      color: #fff;
      border-radius: 24px;
      padding: 28px;
      box-shadow: 0 24px 60px rgba(15, 23, 42, 0.22);
    }}
    .hero h1 {{ margin: 0 0 10px; font-size: 30px; }}
    .hero p {{ margin: 6px 0; line-height: 1.65; color: rgba(255,255,255,0.92); }}
    .grid5, .grid3, .grid2 {{
      display: grid;
      gap: 14px;
      margin-top: 18px;
    }}
    .grid5 {{ grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); }}
    .grid3 {{ grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); }}
    .grid2 {{ grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); }}
    .card, .table-wrap {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 18px;
      box-shadow: var(--shadow);
    }}
    .card {{ padding: 18px; }}
    .eyebrow {{ font-size: 12px; letter-spacing: 0.08em; text-transform: uppercase; color: var(--muted); margin-bottom: 8px; }}
    .big {{ font-size: 30px; font-weight: 700; }}
    .muted {{ color: var(--muted); }}
    .section-title {{ margin: 28px 0 10px; font-size: 18px; }}
    .badge {{
      display: inline-flex;
      align-items: center;
      border-radius: 999px;
      padding: 4px 10px;
      font-size: 12px;
      font-weight: 700;
      color: #fff;
    }}
    .badge.pass {{ background: var(--pass); }}
    .badge.warn {{ background: var(--warn); }}
    .badge.fail {{ background: var(--fail); }}
    .badge.skip, .badge.missing {{ background: var(--skip); }}
    .mono {{
      font-family: "Cascadia Mono", Consolas, monospace;
      white-space: pre-wrap;
      word-break: break-word;
      background: #f8fafc;
      border: 1px solid #e5ebf2;
      border-radius: 10px;
      padding: 10px;
      color: #0f172a;
    }}
    ul {{ margin: 0; padding-left: 18px; }}
    li {{ line-height: 1.7; }}
    .table-wrap {{ overflow: hidden; margin-top: 18px; }}
    table {{ width: 100%; border-collapse: collapse; table-layout: fixed; }}
    th {{
      text-align: left;
      padding: 12px 10px;
      background: #edf4fb;
      border-bottom: 1px solid var(--line);
      font-size: 13px;
      color: #334155;
    }}
    td {{
      vertical-align: top;
      padding: 12px 10px;
      border-bottom: 1px solid #e8eef5;
      font-size: 13px;
      line-height: 1.6;
    }}
    tr:hover {{ background: #f8fbff; }}
    @media (max-width: 900px) {{
      table, thead, tbody, th, td, tr {{ display: block; }}
      thead {{ display: none; }}
      td {{ border-bottom: none; padding-top: 6px; padding-bottom: 6px; }}
      tr {{ display: block; padding: 10px; border-bottom: 1px solid #e8eef5; }}
    }}
  </style>
</head>
<body>
  <div class="wrap">
    <section class="hero">
      <div class="eyebrow">Delivery Validation</div>
      <h1>Horizon Arm ROS2 Validation Report</h1>
      <p>{html.escape(payload["overall_conclusion"])}</p>
      <p>Generated at: {html.escape(payload["generated_at_local"])}</p>
    </section>

    <section class="grid5">
      <div class="card"><div class="eyebrow">Overall</div><div class="big">{html.escape(self._status_label(payload["overall_status"]))}</div></div>
      <div class="card"><div class="eyebrow">Pass</div><div class="big">{summary["PASS"]}</div></div>
      <div class="card"><div class="eyebrow">Warn</div><div class="big">{summary["WARN"]}</div></div>
      <div class="card"><div class="eyebrow">Fail</div><div class="big">{summary["FAIL"]}</div></div>
      <div class="card"><div class="eyebrow">Skip</div><div class="big">{summary["SKIP"]}</div></div>
    </section>

    <h2 class="section-title">Category Summary</h2>
    <section class="grid3">
      {category_cards}
    </section>

    <h2 class="section-title">New Interface Coverage</h2>
    <section class="grid3">
      {coverage_cards}
    </section>

    <h2 class="section-title">Next Actions</h2>
    <section class="grid2">
      <div class="card">
        <div class="eyebrow">Follow Up</div>
        <ul>{action_items}</ul>
      </div>
      <div class="card">
        <div class="eyebrow">How To Read</div>
        <ul>
          <li><strong>PASS</strong>: the endpoint or scenario ran successfully.</li>
          <li><strong>FAIL</strong>: the endpoint or scenario is not ready for delivery confidence.</li>
          <li><strong>SKIP</strong>: the check was intentionally disabled or requires extra hardware/setup.</li>
          <li>The detail column preserves raw service/action feedback for troubleshooting.</li>
        </ul>
      </div>
    </section>

    <h2 class="section-title">Full Result Matrix</h2>
    <section class="table-wrap">
      <table>
        <thead>
          <tr>
            <th style="width: 90px;">Status</th>
            <th style="width: 120px;">Category</th>
            <th style="width: 220px;">Item</th>
            <th style="width: 280px;">Actual Name</th>
            <th>Raw Detail</th>
            <th>Interpretation</th>
            <th style="width: 260px;">Recommendation</th>
          </tr>
        </thead>
        <tbody>
          {table_rows}
        </tbody>
      </table>
    </section>
  </div>
</body>
</html>
"""

    def _render_category_card(self, item: dict[str, Any]) -> str:
        return (
            f"<div class=\"card\">"
            f"<div class=\"eyebrow\">{html.escape(item['category_label'])}</div>"
            f"<div class=\"big\">{item['total']}</div>"
            f"<div class=\"muted\">PASS {item['PASS']} | WARN {item['WARN']} | FAIL {item['FAIL']} | SKIP {item['SKIP']}</div>"
            f"</div>"
        )

    def _render_coverage_card(self, item: dict[str, Any]) -> str:
        status_class = str(item["status"]).lower()
        return (
            f"<div class=\"card\">"
            f"<div class=\"eyebrow\">New Interface</div>"
            f"<div style=\"display:flex;justify-content:space-between;gap:8px;align-items:center;\">"
            f"<strong>{html.escape(item['label'])}</strong>"
            f"<span class=\"badge {html.escape(status_class)}\">{html.escape(item['status_label'])}</span>"
            f"</div>"
            f"<div class=\"mono\" style=\"margin-top:10px;\">{html.escape(str(item['detail']))}</div>"
            f"</div>"
        )

    def _render_result_row(self, item: dict[str, Any]) -> str:
        status_class = item["status"].lower()
        return f"""
<tr>
  <td><span class="badge {status_class}">{html.escape(item["status_label"])}</span></td>
  <td>{html.escape(item["category_label"])}</td>
  <td>{html.escape(item["display_name"])}</td>
  <td><div class="mono">{html.escape(item["actual_name"])}</div></td>
  <td><div class="mono">{html.escape(str(item["detail"]))}</div></td>
  <td>
    <div>{html.escape(item["detail_zh"])}</div>
    <div class="muted" style="margin-top:6px;">{html.escape(item["interpretation"])}</div>
  </td>
  <td>{html.escape(item["recommendation_zh"])}</td>
</tr>
"""

    def _overall_status(self) -> str:
        counts = self._counts()
        if counts["FAIL"] > 0:
            return "FAIL"
        profile = str(self.get_parameter("acceptance_profile").value).strip().lower()
        if profile in ("full", "full_acceptance", "comprehensive"):
            disabled = [
                item
                for item in self._build_test_matrix()
                if not item["enabled"] and item.get("required", True)
            ]
            if disabled:
                return "WARN"
        if counts["WARN"] > 0:
            return "WARN"
        if counts["PASS"] > 0:
            return "PASS"
        return "SKIP"

    def _overall_conclusion(self) -> str:
        counts = self._counts()
        if counts["FAIL"] > 0:
            return "当前环境仍有失败项，处理完成前不能作为最终验收通过结论。"
        profile = str(self.get_parameter("acceptance_profile").value).strip().lower()
        if profile in ("full", "full_acceptance", "comprehensive"):
            disabled = [
                item
                for item in self._build_test_matrix()
                if not item["enabled"] and item.get("required", True)
            ]
            if disabled:
                return (
                    "本次没有执行完整的总验收矩阵。"
                    "在把报告当作最终交付验收依据前，请先补齐未执行项。"
                )
            if not self._io_hardware_available():
                return (
                    "当前环境下可执行的总验收矩阵已通过。"
                    "由于未接入外部 IO 硬件，IO 仅完成 ROS2 接口级验证。"
                )
        if counts["WARN"] > 0:
            return "当前主链路已通过，但仍有提示项需要继续关注。"
        return "本次已执行的验收矩阵在当前环境下通过。"

    def _serialize_result(self, result: CheckResult) -> dict[str, Any]:
        meta = self._result_meta(result)
        return {
            "category": result.category,
            "category_label": self._category_label(result.category),
            "name": result.name,
            "display_name": meta["display_name"],
            "actual_name": meta["actual_name"],
            "status": result.status,
            "status_label": self._status_label(result.status),
            "detail": result.detail,
            "detail_zh": self._detail_zh(result),
            "recommendation": result.recommendation,
            "recommendation_zh": self._recommendation_zh(result),
            "interpretation": self._interpretation(result),
            "context": result.context,
        }

    def _render_html(self, payload: dict[str, Any]) -> str:
        summary = payload["summary"]
        table_rows = "\n".join(self._render_result_row(item) for item in payload["results"])
        return f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Horizon Arm ROS2 总验收报告</title>
  <style>
    :root {{
      --bg: #f2f5f9;
      --panel: #ffffff;
      --text: #132033;
      --muted: #5d6b7c;
      --line: #d9e2ed;
      --pass: #166534;
      --warn: #b45309;
      --fail: #b91c1c;
      --skip: #475569;
      --shadow: 0 16px 40px rgba(15, 23, 42, 0.08);
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      font-family: "Segoe UI", "Microsoft YaHei", sans-serif;
      color: var(--text);
      background: var(--bg);
    }}
    .wrap {{ max-width: 1680px; margin: 0 auto; padding: 18px 14px 28px; }}
    .hero {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 18px 20px;
      box-shadow: var(--shadow);
    }}
    .hero h1 {{ margin: 0 0 8px; font-size: 24px; }}
    .hero p {{ margin: 5px 0; line-height: 1.7; color: var(--text); }}
    .summary {{ background: var(--panel); border: 1px solid var(--line); border-radius: 14px; box-shadow: var(--shadow); padding: 14px 16px; margin-top: 12px; }}
    .summary-line {{ line-height: 1.9; }}
    .muted {{ color: var(--muted); }}
    .badge {{ display: inline-flex; align-items: center; border-radius: 999px; padding: 4px 10px; font-size: 12px; font-weight: 700; color: #fff; }}
    .badge.pass {{ background: var(--pass); }}
    .badge.warn {{ background: var(--warn); }}
    .badge.fail {{ background: var(--fail); }}
    .badge.skip, .badge.missing {{ background: var(--skip); }}
    .mono {{ font-family: "Cascadia Mono", Consolas, monospace; white-space: pre-wrap; word-break: break-word; background: #f8fafc; border: 1px solid #e5ebf2; border-radius: 6px; padding: 8px; color: #0f172a; }}
    .table-wrap {{ overflow-x: auto; margin-top: 12px; background: var(--panel); border: 1px solid var(--line); border-radius: 14px; box-shadow: var(--shadow); }}
    table {{ width: 100%; border-collapse: collapse; table-layout: fixed; }}
    th {{ text-align: left; padding: 10px 8px; background: #edf4fb; border-bottom: 1px solid var(--line); font-size: 13px; color: #334155; position: sticky; top: 0; z-index: 1; }}
    td {{ vertical-align: top; padding: 10px 8px; border-bottom: 1px solid #e8eef5; font-size: 13px; line-height: 1.6; }}
    @media (max-width: 900px) {{
      table, thead, tbody, th, td, tr {{ display: block; }}
      thead {{ display: none; }}
      td {{ border-bottom: none; padding-top: 6px; padding-bottom: 6px; }}
      tr {{ display: block; padding: 10px; border-bottom: 1px solid #e8eef5; }}
    }}
  </style>
</head>
<body>
  <div class="wrap">
    <section class="hero">
      <h1>Horizon Arm ROS2 总验收报告</h1>
      <p>{html.escape(payload["overall_conclusion"])}</p>
      <p>验收模式：{html.escape(str(payload["acceptance_profile"]))} | 允许真实副作用：{html.escape(str(payload["allow_hardware_side_effects"]))}</p>
      <p>生成时间：{html.escape(payload["generated_at_local"])}</p>
    </section>
    <section class="summary">
      <div class="summary-line">总体状态：<strong>{html.escape(self._status_label(payload["overall_status"]))}</strong></div>
      <div class="summary-line">通过：{summary["PASS"]}　提示：{summary["WARN"]}　失败：{summary["FAIL"]}　跳过：{summary["SKIP"]}</div>
      <div class="summary-line">说明：本报告按执行顺序列出全部检查项；新增接口测试项已直接并入主表，参数栏为空的旧问题已改为明确中文说明。</div>
    </section>
    <section class="table-wrap">
      <table>
        <thead>
          <tr>
            <th style="width: 80px;">状态</th>
            <th style="width: 110px;">分类</th>
            <th style="width: 200px;">检查项</th>
            <th style="width: 220px;">实际接口/对象</th>
            <th style="width: 290px;">请求参数（中文）</th>
            <th>原始结果 / 返回参数</th>
            <th style="width: 260px;">判定说明 / 正常参考</th>
            <th style="width: 220px;">建议</th>
          </tr>
        </thead>
        <tbody>{table_rows}</tbody>
      </table>
    </section>
  </div>
</body>
</html>
"""

    def _render_matrix_row(self, item: dict[str, Any]) -> str:
        enabled_label = "已开启" if item["enabled"] else "未开启"
        status_class = "pass" if item["enabled"] else "skip"
        side_effect = "是" if item["side_effect"] else "否"
        return (
            "<tr>"
            f"<td>{html.escape(item['label'])}</td>"
            f"<td><div class=\"mono\">{html.escape(item['parameter'])}</div></td>"
            f"<td><span class=\"badge {status_class}\">{enabled_label}</span></td>"
            f"<td>{html.escape(item['module'])}</td>"
            f"<td>{side_effect}</td>"
            f"<td>{html.escape(item['reason'])}</td>"
            "</tr>"
        )

    def _render_result_row(self, item: dict[str, Any]) -> str:
        status_class = item["status"].lower()
        context_zh = html.escape(self._context_zh(item))
        reference_hint = html.escape(self._reference_hint(item))
        return f"""
<tr>
  <td><span class="badge {status_class}">{html.escape(item["status_label"])}</span></td>
  <td>{html.escape(item["category_label"])}</td>
  <td>{html.escape(item["display_name"])}</td>
  <td><div class="mono">{html.escape(item["actual_name"])}</div></td>
  <td><div class="mono">{context_zh}</div></td>
  <td><div class="mono">{html.escape(str(item["detail"]))}</div></td>
  <td>
    <div>{reference_hint}</div>
    <div class="muted" style="margin-top:6px;">{html.escape(item["interpretation"])}</div>
  </td>
  <td>
    <div>{html.escape(item["recommendation_zh"])}</div>
  </td>
</tr>
"""

    def _print_summary(self) -> None:
        counts = self._counts()
        print("\n=== Horizon Arm 系统验收汇总 ===")
        print(
            f"PASS={counts.get('PASS', 0)} "
            f"WARN={counts.get('WARN', 0)} "
            f"FAIL={counts.get('FAIL', 0)} "
            f"SKIP={counts.get('SKIP', 0)}"
        )
        for result in self._results:
            print(
                f"[{result.status:<4}] {result.category} :: {result.name} :: {result.detail}"
            )
            if result.recommendation:
                print(f"        recommendation: {result.recommendation}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SystemCheckNode()
    exit_code = 1
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()

