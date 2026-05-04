from __future__ import annotations

import dataclasses
import json
from typing import Any, List, Optional, Sequence

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
from std_srvs.srv import Trigger

from .common import (
    DEFAULT_JOINT_NAMES,
    build_trajectory_goal_deg,
    build_trajectory_goal_rad,
    load_preset_config,
)


@dataclasses.dataclass(frozen=True)
class TriggerCallResult:
    success: bool
    message: str


@dataclasses.dataclass(frozen=True)
class DigitalOutputCallResult:
    success: bool
    message: str


@dataclasses.dataclass(frozen=True)
class GripperCallResult:
    success: bool
    message: str


@dataclasses.dataclass(frozen=True)
class SimpleServiceCallResult:
    success: bool
    message: str


@dataclasses.dataclass(frozen=True)
class JsonServiceCallResult:
    success: bool
    message: str
    payload_json: str


@dataclasses.dataclass(frozen=True)
class TeachingProgramResult:
    success: bool
    message: str
    result_json: str


@dataclasses.dataclass(frozen=True)
class TrajectoryExecutionResult:
    success: bool
    error_code: int
    error_string: str


@dataclasses.dataclass(frozen=True)
class InstructionExecutionResult:
    success: bool
    message: str


class HorizonArmRosSdk:
    """Blocking helper API for developer scripts built on the ROS2 package."""

    def __init__(
        self,
        node: Node,
        *,
        joint_names: Optional[Sequence[str]] = None,
        trajectory_action_name: str = "/horizon_arm_controller/follow_joint_trajectory",
        instruction_action_name: str = "/horizon_arm/run_instruction",
        status_topic: str = "/horizon_arm/status",
        enable_service_name: str = "/horizon_arm/enable",
        disable_service_name: str = "/horizon_arm/disable",
        estop_service_name: str = "/horizon_arm/emergency_stop",
        digital_output_service_name: str = "/horizon_arm/set_digital_output",
        gripper_service_name: str = "/horizon_arm/set_gripper_state",
        visual_grasp_service_name: str = "/horizon_arm/visual_grasp",
        visual_grasp_ex_service_name: str = "/horizon_arm/visual_grasp_ex",
        vision_config_service_name: str = "/horizon_arm/vision_config",
        pick_hsv_service_name: str = "/horizon_arm/pick_hsv",
        detect_target_service_name: str = "/horizon_arm/detect_target",
        follow_grasp_service_name: str = "/horizon_arm/follow_grasp_control",
        follow_target_service_name: str = "/horizon_arm/follow_target",
        joycon_service_name: str = "/horizon_arm/joycon_control",
        joycon_advanced_service_name: str = "/horizon_arm/joycon_advanced_control",
        teach_jog_service_name: str = "/horizon_arm/teach_jog",
        teaching_program_action_name: str = "/horizon_arm/teaching_program",
        embodied_service_name: str = "/horizon_arm/embodied_instruction",
        embodied_command_service_name: str = "/horizon_arm/embodied_command",
        preset_config_path: str = "",
    ) -> None:
        self._node = node
        self._joint_names = list(joint_names or DEFAULT_JOINT_NAMES)
        self._preset_config_path = preset_config_path
        self._preset_config: Optional[dict[str, Any]] = None
        self._latest_status: Optional[ArmStatus] = None

        self._motion_client = ActionClient(
            node,
            FollowJointTrajectory,
            trajectory_action_name,
        )
        self._instruction_client = ActionClient(
            node,
            RunInstruction,
            instruction_action_name,
        )
        self._teaching_program_client = ActionClient(
            node,
            TeachingProgram,
            teaching_program_action_name,
        )
        self._enable_client = node.create_client(Trigger, enable_service_name)
        self._disable_client = node.create_client(Trigger, disable_service_name)
        self._estop_client = node.create_client(Trigger, estop_service_name)
        self._digital_output_client = node.create_client(
            SetDigitalOutput,
            digital_output_service_name,
        )
        self._gripper_client = node.create_client(
            SetGripperState,
            gripper_service_name,
        )
        self._visual_grasp_client = node.create_client(
            VisualGrasp,
            visual_grasp_service_name,
        )
        self._visual_grasp_ex_client = node.create_client(
            VisualGraspEx,
            visual_grasp_ex_service_name,
        )
        self._vision_config_client = node.create_client(
            VisionConfig,
            vision_config_service_name,
        )
        self._pick_hsv_client = node.create_client(
            PickHSV,
            pick_hsv_service_name,
        )
        self._detect_target_client = node.create_client(
            DetectTarget,
            detect_target_service_name,
        )
        self._follow_grasp_client = node.create_client(
            FollowGraspControl,
            follow_grasp_service_name,
        )
        self._follow_target_client = node.create_client(
            FollowTarget,
            follow_target_service_name,
        )
        self._joycon_client = node.create_client(
            JoyconControl,
            joycon_service_name,
        )
        self._joycon_advanced_client = node.create_client(
            JoyconAdvancedControl,
            joycon_advanced_service_name,
        )
        self._teach_jog_client = node.create_client(
            TeachJog,
            teach_jog_service_name,
        )
        self._embodied_client = node.create_client(
            EmbodiedInstruction,
            embodied_service_name,
        )
        self._embodied_command_client = node.create_client(
            EmbodiedCommand,
            embodied_command_service_name,
        )
        self._status_sub = node.create_subscription(
            ArmStatus,
            status_topic,
            self._on_status,
            10,
        )

    @property
    def latest_status(self) -> Optional[ArmStatus]:
        return self._latest_status

    def wait_until_ready(
        self,
        *,
        timeout_sec: float = 5.0,
        include_instruction: bool = False,
        include_digital_output: bool = False,
        include_gripper: bool = False,
        include_extended_wrappers: bool = False,
    ) -> bool:
        timeout_sec = max(0.1, float(timeout_sec))
        ok = self._motion_client.wait_for_server(timeout_sec=timeout_sec)
        ok = ok and self._enable_client.wait_for_service(timeout_sec=timeout_sec)
        ok = ok and self._disable_client.wait_for_service(timeout_sec=timeout_sec)
        ok = ok and self._estop_client.wait_for_service(timeout_sec=timeout_sec)
        if include_instruction:
            ok = ok and self._instruction_client.wait_for_server(timeout_sec=timeout_sec)
        if include_digital_output:
            ok = ok and self._digital_output_client.wait_for_service(
                timeout_sec=timeout_sec
            )
        if include_gripper:
            ok = ok and self._gripper_client.wait_for_service(timeout_sec=timeout_sec)
        if include_extended_wrappers:
            ok = ok and self._visual_grasp_client.wait_for_service(timeout_sec=timeout_sec)
            ok = ok and self._visual_grasp_ex_client.wait_for_service(timeout_sec=timeout_sec)
            ok = ok and self._vision_config_client.wait_for_service(timeout_sec=timeout_sec)
            ok = ok and self._pick_hsv_client.wait_for_service(timeout_sec=timeout_sec)
            ok = ok and self._detect_target_client.wait_for_service(timeout_sec=timeout_sec)
            ok = ok and self._follow_grasp_client.wait_for_service(timeout_sec=timeout_sec)
            ok = ok and self._follow_target_client.wait_for_service(timeout_sec=timeout_sec)
            ok = ok and self._joycon_client.wait_for_service(timeout_sec=timeout_sec)
            ok = ok and self._joycon_advanced_client.wait_for_service(timeout_sec=timeout_sec)
            ok = ok and self._teach_jog_client.wait_for_service(timeout_sec=timeout_sec)
            ok = ok and self._embodied_client.wait_for_service(timeout_sec=timeout_sec)
            ok = ok and self._embodied_command_client.wait_for_service(timeout_sec=timeout_sec)
        return bool(ok)

    def enable(self, *, timeout_sec: float = 5.0) -> TriggerCallResult:
        return self._call_trigger(self._enable_client, timeout_sec=timeout_sec)

    def disable(self, *, timeout_sec: float = 5.0) -> TriggerCallResult:
        return self._call_trigger(self._disable_client, timeout_sec=timeout_sec)

    def emergency_stop(self, *, timeout_sec: float = 5.0) -> TriggerCallResult:
        return self._call_trigger(self._estop_client, timeout_sec=timeout_sec)

    def move_joints_deg(
        self,
        joints_deg: Sequence[float] | Sequence[Sequence[float]],
        *,
        duration_sec: float = 2.0,
        timeout_sec: float = 15.0,
    ) -> TrajectoryExecutionResult:
        goal = build_trajectory_goal_deg(
            self._joint_names,
            joints_deg,
            duration_sec,
        )
        return self._execute_motion_goal(goal, timeout_sec=timeout_sec)

    def move_joints_rad(
        self,
        joints_rad: Sequence[float] | Sequence[Sequence[float]],
        *,
        duration_sec: float = 2.0,
        timeout_sec: float = 15.0,
    ) -> TrajectoryExecutionResult:
        goal = build_trajectory_goal_rad(
            self._joint_names,
            joints_rad,
            duration_sec,
        )
        return self._execute_motion_goal(goal, timeout_sec=timeout_sec)

    def execute_preset(
        self,
        preset_name: str,
        *,
        timeout_sec: float = 15.0,
    ) -> TrajectoryExecutionResult:
        presets = self._load_presets()
        if preset_name not in presets:
            raise KeyError(f"unknown preset: {preset_name}")
        preset = presets[preset_name]
        joints = preset.get("joints")
        if joints is None:
            raise ValueError(f"preset {preset_name} has no joints field")
        duration = float(preset.get("duration", 2.0))
        return self.move_joints_deg(
            joints,
            duration_sec=duration,
            timeout_sec=timeout_sec,
        )

    def set_digital_output(
        self,
        channel: int,
        state: bool,
        *,
        timeout_sec: float = 5.0,
    ) -> DigitalOutputCallResult:
        if not self._digital_output_client.wait_for_service(timeout_sec=timeout_sec):
            return DigitalOutputCallResult(
                success=False,
                message="digital output service is not ready",
            )
        request = SetDigitalOutput.Request()
        request.channel = int(channel)
        request.state = bool(state)
        future = self._digital_output_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return DigitalOutputCallResult(
            success=bool(response.success),
            message=str(response.message),
        )

    def set_gripper_state(
        self,
        *,
        open: bool,
        current_ma: int = 1200,
        timeout_sec: float = 5.0,
    ) -> GripperCallResult:
        if not self._gripper_client.wait_for_service(timeout_sec=timeout_sec):
            return GripperCallResult(
                success=False,
                message="gripper service is not ready",
            )
        request = SetGripperState.Request()
        request.open = bool(open)
        request.current_ma = max(0, int(current_ma))
        future = self._gripper_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return GripperCallResult(
            success=bool(response.success),
            message=str(response.message),
        )

    def open_gripper(
        self,
        *,
        current_ma: int = 1200,
        timeout_sec: float = 5.0,
    ) -> GripperCallResult:
        return self.set_gripper_state(
            open=True,
            current_ma=current_ma,
            timeout_sec=timeout_sec,
        )

    def close_gripper(
        self,
        *,
        current_ma: int = 1200,
        timeout_sec: float = 5.0,
    ) -> GripperCallResult:
        return self.set_gripper_state(
            open=False,
            current_ma=current_ma,
            timeout_sec=timeout_sec,
        )

    def visual_grasp(
        self,
        *,
        u: float = 0.0,
        v: float = 0.0,
        bbox: Sequence[float] | None = None,
        dry_run: bool = False,
        timeout_sec: float = 10.0,
    ) -> SimpleServiceCallResult:
        if not self._visual_grasp_client.wait_for_service(timeout_sec=timeout_sec):
            return SimpleServiceCallResult(False, "visual grasp service is not ready")
        request = VisualGrasp.Request()
        request.dry_run = bool(dry_run)
        request.use_bbox = bbox is not None
        request.u = float(u)
        request.v = float(v)
        if bbox is not None:
            values = list(bbox)
            if len(values) < 4:
                raise ValueError("bbox must contain x1, y1, x2, y2")
            request.x1, request.y1, request.x2, request.y2 = [float(v) for v in values[:4]]
        future = self._visual_grasp_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return SimpleServiceCallResult(bool(response.success), str(response.message))

    def visual_grasp_health(self, *, timeout_sec: float = 5.0) -> SimpleServiceCallResult:
        if not self._visual_grasp_client.wait_for_service(timeout_sec=timeout_sec):
            return SimpleServiceCallResult(False, "visual grasp service is not ready")
        request = VisualGrasp.Request()
        request.dry_run = True
        future = self._visual_grasp_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return SimpleServiceCallResult(bool(response.success), str(response.message))

    def follow_grasp_status(self, *, timeout_sec: float = 5.0) -> SimpleServiceCallResult:
        if not self._follow_grasp_client.wait_for_service(timeout_sec=timeout_sec):
            return SimpleServiceCallResult(False, "follow grasp service is not ready")
        request = FollowGraspControl.Request()
        request.command = "status"
        future = self._follow_grasp_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return SimpleServiceCallResult(bool(response.success), str(response.message))

    def visual_grasp_ex(
        self,
        *,
        mode: str = "click",
        pipeline: str = "click",
        target_class: str = "",
        u: float = 0.0,
        v: float = 0.0,
        bbox: Sequence[float] | None = None,
        use_hsv: bool = False,
        use_depth: bool = True,
        dry_run: bool = False,
        z_offset_m: float = 0.0,
        approach_height_m: float = 0.08,
        grasp_depth_m: float = 0.02,
        pre_grasp_rpy: Sequence[float] | None = None,
        options_json: str = "",
        timeout_sec: float = 10.0,
    ) -> JsonServiceCallResult:
        if not self._visual_grasp_ex_client.wait_for_service(timeout_sec=timeout_sec):
            return JsonServiceCallResult(False, "visual grasp ex service is not ready", "{}")
        request = VisualGraspEx.Request()
        request.mode = str(mode)
        request.pipeline = str(pipeline)
        request.target_class = str(target_class)
        request.u = float(u)
        request.v = float(v)
        request.use_click = bbox is None
        request.use_bbox = bbox is not None
        request.use_hsv = bool(use_hsv)
        request.use_depth = bool(use_depth)
        request.dry_run = bool(dry_run)
        request.z_offset_m = float(z_offset_m)
        request.approach_height_m = float(approach_height_m)
        request.grasp_depth_m = float(grasp_depth_m)
        request.options_json = str(options_json)
        if pre_grasp_rpy is not None:
            request.pre_grasp_rpy = [float(v) for v in list(pre_grasp_rpy)[:3]]
        if bbox is not None:
            values = list(bbox)
            if len(values) < 4:
                raise ValueError("bbox must contain x1, y1, x2, y2")
            request.x1, request.y1, request.x2, request.y2 = [float(v) for v in values[:4]]
        future = self._visual_grasp_ex_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return JsonServiceCallResult(
            bool(response.success),
            str(response.message),
            str(response.result_json),
        )

    def configure_vision(
        self,
        *,
        command: str = "set",
        pipeline: str = "",
        target_class: str = "",
        conf_thres: float = 0.0,
        iou_thres: float = 0.0,
        interval_sec: float = 0.0,
        depth_min_m: float = 0.0,
        depth_max_m: float = 0.0,
        pixel_to_mm_scale: float = 0.0,
        model_path: str = "",
        camera_name: str = "",
        hsv: Sequence[int] | None = None,
        options_json: str = "",
        timeout_sec: float = 5.0,
    ) -> JsonServiceCallResult:
        if not self._vision_config_client.wait_for_service(timeout_sec=timeout_sec):
            return JsonServiceCallResult(False, "vision config service is not ready", "{}")
        request = VisionConfig.Request()
        request.command = str(command)
        request.pipeline = str(pipeline)
        request.target_class = str(target_class)
        request.conf_thres = float(conf_thres)
        request.iou_thres = float(iou_thres)
        request.interval_sec = float(interval_sec)
        request.depth_min_m = float(depth_min_m)
        request.depth_max_m = float(depth_max_m)
        request.pixel_to_mm_scale = float(pixel_to_mm_scale)
        request.model_path = str(model_path)
        request.camera_name = str(camera_name)
        request.options_json = str(options_json)
        if hsv is not None:
            values = list(hsv)
            if len(values) != 6:
                raise ValueError("hsv must be [h_min,h_max,s_min,s_max,v_min,v_max]")
            (
                request.hsv_h_min,
                request.hsv_h_max,
                request.hsv_s_min,
                request.hsv_s_max,
                request.hsv_v_min,
                request.hsv_v_max,
            ) = [int(v) for v in values]
        future = self._vision_config_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return JsonServiceCallResult(
            bool(response.success),
            str(response.message),
            str(response.config_json),
        )

    def pick_hsv(
        self,
        u: float,
        v: float,
        *,
        window_size: int = 9,
        use_depth_filter: bool = True,
        depth_min_m: float = 0.15,
        depth_max_m: float = 1.2,
        timeout_sec: float = 5.0,
    ) -> JsonServiceCallResult:
        if not self._pick_hsv_client.wait_for_service(timeout_sec=timeout_sec):
            return JsonServiceCallResult(False, "pick HSV service is not ready", "{}")
        request = PickHSV.Request()
        request.u = float(u)
        request.v = float(v)
        request.window_size = int(window_size)
        request.use_depth_filter = bool(use_depth_filter)
        request.depth_min_m = float(depth_min_m)
        request.depth_max_m = float(depth_max_m)
        future = self._pick_hsv_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        payload = {
            "h": response.h,
            "s": response.s,
            "v": response.v,
            "h_min": response.h_min,
            "h_max": response.h_max,
            "s_min": response.s_min,
            "s_max": response.s_max,
            "v_min": response.v_min,
            "v_max": response.v_max,
            "depth_m": response.depth_m,
        }
        return JsonServiceCallResult(
            bool(response.success),
            str(response.message),
            json.dumps(payload, ensure_ascii=False),
        )

    def detect_target(
        self,
        *,
        pipeline: str = "yolo",
        target_class: str = "",
        conf_thres: float = 0.5,
        use_hsv: bool = False,
        use_depth: bool = True,
        depth_min_m: float = 0.15,
        depth_max_m: float = 1.2,
        timeout_sec: float = 10.0,
    ) -> JsonServiceCallResult:
        if not self._detect_target_client.wait_for_service(timeout_sec=timeout_sec):
            return JsonServiceCallResult(False, "detect target service is not ready", "{}")
        request = DetectTarget.Request()
        request.pipeline = str(pipeline)
        request.target_class = str(target_class)
        request.conf_thres = float(conf_thres)
        request.use_hsv = bool(use_hsv)
        request.use_depth = bool(use_depth)
        request.depth_min_m = float(depth_min_m)
        request.depth_max_m = float(depth_max_m)
        future = self._detect_target_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        payload = {
            "count": response.count,
            "bboxes": list(response.bboxes),
            "centers": list(response.centers),
            "scores": list(response.scores),
            "class_names": list(response.class_names),
            "depths_m": list(response.depths_m),
        }
        return JsonServiceCallResult(
            bool(response.success),
            str(response.message),
            json.dumps(payload, ensure_ascii=False),
        )

    def follow_grasp_control(
        self,
        command: str = "status",
        *,
        target_class: str = "",
        conf_thres: float = 0.0,
        interval_sec: float = 0.0,
        timeout_sec: float = 5.0,
    ) -> JsonServiceCallResult:
        if not self._follow_grasp_client.wait_for_service(timeout_sec=timeout_sec):
            return JsonServiceCallResult(False, "follow grasp service is not ready", "{}")
        request = FollowGraspControl.Request()
        request.command = str(command)
        request.target_class = str(target_class)
        request.conf_thres = float(conf_thres)
        request.interval_sec = float(interval_sec)
        future = self._follow_grasp_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        payload = {
            "running": bool(response.running),
            "command": str(command),
            "target_class": str(target_class),
            "conf_thres": float(conf_thres),
            "interval_sec": float(interval_sec),
        }
        return JsonServiceCallResult(
            bool(response.success),
            str(response.message),
            json.dumps(payload, ensure_ascii=False),
        )

    def follow_target(
        self,
        command: str = "status",
        *,
        mode: str = "yolo",
        pipeline: str = "yolo",
        target_class: str = "",
        conf_thres: float = 0.5,
        interval_sec: float = 0.1,
        follow_distance_m: float = 0.35,
        deadband_px: float = 0.0,
        max_linear_speed: float = 0.0,
        max_angular_speed: float = 0.0,
        use_depth: bool = True,
        auto_grasp: bool = False,
        options_json: str = "",
        timeout_sec: float = 5.0,
    ) -> JsonServiceCallResult:
        if not self._follow_target_client.wait_for_service(timeout_sec=timeout_sec):
            return JsonServiceCallResult(False, "follow target service is not ready", "{}")
        request = FollowTarget.Request()
        request.command = str(command)
        request.mode = str(mode)
        request.pipeline = str(pipeline)
        request.target_class = str(target_class)
        request.conf_thres = float(conf_thres)
        request.interval_sec = float(interval_sec)
        request.follow_distance_m = float(follow_distance_m)
        request.deadband_px = float(deadband_px)
        request.max_linear_speed = float(max_linear_speed)
        request.max_angular_speed = float(max_angular_speed)
        request.use_depth = bool(use_depth)
        request.auto_grasp = bool(auto_grasp)
        request.options_json = str(options_json)
        future = self._follow_target_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return JsonServiceCallResult(
            bool(response.success),
            str(response.message),
            str(response.state_json),
        )

    def joycon_control(
        self,
        command: str = "status",
        *,
        timeout_sec: float = 5.0,
    ) -> SimpleServiceCallResult:
        if not self._joycon_client.wait_for_service(timeout_sec=timeout_sec):
            return SimpleServiceCallResult(False, "joycon service is not ready")
        request = JoyconControl.Request()
        request.command = str(command)
        future = self._joycon_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return SimpleServiceCallResult(bool(response.success), str(response.message))

    def joycon_status(self, *, timeout_sec: float = 5.0) -> SimpleServiceCallResult:
        return self.joycon_control("status", timeout_sec=timeout_sec)

    def joycon_advanced(
        self,
        command: str = "status",
        *,
        mode: str = "",
        attitude_mode: str = "",
        enabled: bool = False,
        dual_arm: bool = False,
        preferred_side: str = "",
        speed_index: int = 0,
        speed_levels: Sequence[float] | None = None,
        stick_deadzone: int = 0,
        cartesian_position_step: float = 0.0,
        cartesian_rotation_step: float = 0.0,
        cartesian_max_speed: float = 0.0,
        cartesian_max_angular_speed: float = 0.0,
        joint_angle_step: float = 0.0,
        joint_max_speed: int = 0,
        joint_acceleration: int = 0,
        joint_deceleration: int = 0,
        attitude_position_speed_mm_s: float = 0.0,
        attitude_mode2_position_speed_mm_s: float = 0.0,
        attitude_rotation_gain: float = 0.0,
        attitude_mode2_rotation_gain: float = 0.0,
        attitude_joint_max_speed: float = 0.0,
        attitude_joint_acceleration: float = 0.0,
        attitude_joint_deceleration: float = 0.0,
        workspace_min_radius: float = 0.0,
        workspace_max_radius: float = 0.0,
        workspace_min_z: float = 0.0,
        workspace_max_z: float = 0.0,
        timeout_sec: float = 5.0,
    ) -> JsonServiceCallResult:
        if not self._joycon_advanced_client.wait_for_service(timeout_sec=timeout_sec):
            return JsonServiceCallResult(False, "joycon advanced service is not ready", "{}")
        request = JoyconAdvancedControl.Request()
        request.command = str(command)
        request.mode = str(mode)
        request.attitude_mode = str(attitude_mode)
        request.enabled = bool(enabled)
        request.dual_arm = bool(dual_arm)
        request.preferred_side = str(preferred_side)
        request.speed_index = int(speed_index)
        if speed_levels is not None:
            request.speed_levels = [float(v) for v in speed_levels]
        request.stick_deadzone = int(stick_deadzone)
        request.cartesian_position_step = float(cartesian_position_step)
        request.cartesian_rotation_step = float(cartesian_rotation_step)
        request.cartesian_max_speed = float(cartesian_max_speed)
        request.cartesian_max_angular_speed = float(cartesian_max_angular_speed)
        request.joint_angle_step = float(joint_angle_step)
        request.joint_max_speed = int(joint_max_speed)
        request.joint_acceleration = int(joint_acceleration)
        request.joint_deceleration = int(joint_deceleration)
        request.attitude_position_speed_mm_s = float(attitude_position_speed_mm_s)
        request.attitude_mode2_position_speed_mm_s = float(
            attitude_mode2_position_speed_mm_s
        )
        request.attitude_rotation_gain = float(attitude_rotation_gain)
        request.attitude_mode2_rotation_gain = float(attitude_mode2_rotation_gain)
        request.attitude_joint_max_speed = float(attitude_joint_max_speed)
        request.attitude_joint_acceleration = float(attitude_joint_acceleration)
        request.attitude_joint_deceleration = float(attitude_joint_deceleration)
        request.workspace_min_radius = float(workspace_min_radius)
        request.workspace_max_radius = float(workspace_max_radius)
        request.workspace_min_z = float(workspace_min_z)
        request.workspace_max_z = float(workspace_max_z)
        future = self._joycon_advanced_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return JsonServiceCallResult(
            bool(response.success),
            str(response.message),
            str(response.status_json),
        )

    def teach_jog(
        self,
        command: str = "status",
        *,
        frame: str = "",
        axis: str = "",
        joint_index: int = 0,
        delta: float = 0.0,
        joint_angles: Sequence[float] | None = None,
        position: Sequence[float] | None = None,
        orientation: Sequence[float] | None = None,
        interpolation_type: str = "",
        max_speed: float = 0.0,
        acceleration: float = 0.0,
        deceleration: float = 0.0,
        linear_velocity: float = 0.0,
        angular_velocity: float = 0.0,
        linear_acceleration: float = 0.0,
        angular_acceleration: float = 0.0,
        joint_max_velocities: Sequence[float] | None = None,
        joint_max_accelerations: Sequence[float] | None = None,
        dry_run: bool = False,
        timeout_sec: float = 10.0,
    ) -> JsonServiceCallResult:
        if not self._teach_jog_client.wait_for_service(timeout_sec=timeout_sec):
            return JsonServiceCallResult(False, "teach jog service is not ready", "{}")
        request = TeachJog.Request()
        request.command = str(command)
        request.frame = str(frame)
        request.axis = str(axis)
        request.joint_index = int(joint_index)
        request.delta = float(delta)
        if joint_angles is not None:
            request.joint_angles = [float(v) for v in joint_angles]
        if position is not None:
            request.position = [float(v) for v in position]
        if orientation is not None:
            request.orientation = [float(v) for v in orientation]
        request.interpolation_type = str(interpolation_type)
        request.max_speed = float(max_speed)
        request.acceleration = float(acceleration)
        request.deceleration = float(deceleration)
        request.linear_velocity = float(linear_velocity)
        request.angular_velocity = float(angular_velocity)
        request.linear_acceleration = float(linear_acceleration)
        request.angular_acceleration = float(angular_acceleration)
        if joint_max_velocities is not None:
            request.joint_max_velocities = [float(v) for v in joint_max_velocities]
        if joint_max_accelerations is not None:
            request.joint_max_accelerations = [
                float(v) for v in joint_max_accelerations
            ]
        request.dry_run = bool(dry_run)
        future = self._teach_jog_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return JsonServiceCallResult(
            bool(response.success),
            str(response.message),
            str(response.detail_json),
        )

    def teach_jog_joint(
        self,
        joint_index: int,
        delta_deg: float,
        *,
        dry_run: bool = False,
        timeout_sec: float = 10.0,
    ) -> JsonServiceCallResult:
        return self.teach_jog(
            "joint_jog",
            joint_index=joint_index,
            delta=delta_deg,
            dry_run=dry_run,
            timeout_sec=timeout_sec,
        )

    def run_teaching_program(
        self,
        program_path: str = "",
        *,
        command: str = "run",
        program_name: str = "",
        program_json: str = "",
        use_saved_params: bool = False,
        dry_run: bool = False,
        timeout_sec: float = 60.0,
    ) -> TeachingProgramResult:
        if not self._teaching_program_client.wait_for_server(timeout_sec=timeout_sec):
            return TeachingProgramResult(False, "teaching program action is not ready", "{}")
        goal = TeachingProgram.Goal()
        goal.command = str(command)
        goal.program_name = str(program_name)
        goal.program_path = str(program_path)
        goal.program_json = str(program_json)
        goal.use_saved_params = bool(use_saved_params)
        goal.dry_run = bool(dry_run)
        send_future = self._teaching_program_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(send_future, timeout_sec=timeout_sec)
        if not goal_handle.accepted:
            return TeachingProgramResult(False, "teaching program goal was rejected", "{}")
        result_future = goal_handle.get_result_async()
        wrapped_result = self._wait_for_future(result_future, timeout_sec=timeout_sec)
        result = wrapped_result.result
        return TeachingProgramResult(
            bool(result.success),
            str(result.message),
            str(result.result_json),
        )

    def embodied_health(self, *, timeout_sec: float = 5.0) -> SimpleServiceCallResult:
        if not self._embodied_client.wait_for_service(timeout_sec=timeout_sec):
            return SimpleServiceCallResult(False, "embodied service is not ready")
        request = EmbodiedInstruction.Request()
        request.command = "health"
        request.stream = False
        future = self._embodied_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return SimpleServiceCallResult(bool(response.success), str(response.message))

    def embodied_instruction(
        self,
        instruction: str = "health",
        *,
        stream: bool = False,
        timeout_sec: float = 30.0,
    ) -> JsonServiceCallResult:
        if not self._embodied_client.wait_for_service(timeout_sec=timeout_sec):
            return JsonServiceCallResult(False, "embodied service is not ready", "{}")
        request = EmbodiedInstruction.Request()
        request.command = str(instruction)
        request.stream = bool(stream)
        future = self._embodied_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return JsonServiceCallResult(
            bool(response.success),
            str(response.message),
            str(response.result_json),
        )

    def embodied_command(
        self,
        command: str = "health",
        *,
        instruction: str = "",
        stream: bool = False,
        provider: str = "",
        model: str = "",
        control_mode: str = "",
        options_json: str = "",
        timeout_sec: float = 30.0,
    ) -> JsonServiceCallResult:
        if not self._embodied_command_client.wait_for_service(timeout_sec=timeout_sec):
            return JsonServiceCallResult(False, "embodied command service is not ready", "{}")
        request = EmbodiedCommand.Request()
        request.command = str(command)
        request.instruction = str(instruction)
        request.stream = bool(stream)
        request.provider = str(provider)
        request.model = str(model)
        request.control_mode = str(control_mode)
        request.options_json = str(options_json)
        future = self._embodied_command_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return JsonServiceCallResult(
            bool(response.success),
            str(response.message),
            str(response.result_json),
        )

    def run_instruction(
        self,
        instruction: str,
        *,
        timeout_sec: float = 20.0,
    ) -> InstructionExecutionResult:
        if not self._instruction_client.wait_for_server(timeout_sec=timeout_sec):
            return InstructionExecutionResult(
                success=False,
                message="instruction action server is not ready",
            )
        goal = RunInstruction.Goal()
        goal.instruction = str(instruction)
        send_future = self._instruction_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(send_future, timeout_sec=timeout_sec)
        if not goal_handle.accepted:
            return InstructionExecutionResult(
                success=False,
                message="instruction goal was rejected",
            )
        result_future = goal_handle.get_result_async()
        wrapped_result = self._wait_for_future(result_future, timeout_sec=timeout_sec)
        result = wrapped_result.result
        return InstructionExecutionResult(
            success=bool(result.success),
            message=str(result.message),
        )

    def _load_presets(self) -> dict[str, Any]:
        if self._preset_config is None:
            self._preset_config = load_preset_config(self._preset_config_path)
        return self._preset_config

    def _call_trigger(
        self,
        client,
        *,
        timeout_sec: float,
    ) -> TriggerCallResult:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return TriggerCallResult(success=False, message="service is not ready")
        future = client.call_async(Trigger.Request())
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return TriggerCallResult(
            success=bool(response.success),
            message=str(response.message),
        )

    def _execute_motion_goal(
        self,
        goal: FollowJointTrajectory.Goal,
        *,
        timeout_sec: float,
    ) -> TrajectoryExecutionResult:
        if not self._motion_client.wait_for_server(timeout_sec=timeout_sec):
            return TrajectoryExecutionResult(
                success=False,
                error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                error_string="follow_joint_trajectory action server is not ready",
            )
        send_future = self._motion_client.send_goal_async(goal)
        goal_handle = self._wait_for_future(send_future, timeout_sec=timeout_sec)
        if not goal_handle.accepted:
            return TrajectoryExecutionResult(
                success=False,
                error_code=FollowJointTrajectory.Result.INVALID_GOAL,
                error_string="trajectory goal was rejected",
            )
        result_future = goal_handle.get_result_async()
        wrapped_result = self._wait_for_future(result_future, timeout_sec=timeout_sec)
        result = wrapped_result.result
        return TrajectoryExecutionResult(
            success=(
                result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            ),
            error_code=int(result.error_code),
            error_string=str(result.error_string),
        )

    def _wait_for_future(self, future, *, timeout_sec: float):
        rclpy.spin_until_future_complete(
            self._node,
            future,
            timeout_sec=max(0.1, float(timeout_sec)),
        )
        if not future.done():
            raise TimeoutError("ROS2 call timed out")
        return future.result()

    def _on_status(self, msg: ArmStatus) -> None:
        self._latest_status = msg
