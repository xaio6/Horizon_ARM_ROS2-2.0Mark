from __future__ import annotations

import dataclasses
from typing import Any, List, Optional, Sequence

from control_msgs.action import FollowJointTrajectory
from horizon_arm_interfaces.action import RunInstruction
from horizon_arm_interfaces.msg import ArmStatus
from horizon_arm_interfaces.srv import (
    EmbodiedInstruction,
    FollowGraspControl,
    JoyconControl,
    SetDigitalOutput,
    SetGripperState,
    VisualGrasp,
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
        follow_grasp_service_name: str = "/horizon_arm/follow_grasp_control",
        joycon_service_name: str = "/horizon_arm/joycon_control",
        embodied_service_name: str = "/horizon_arm/embodied_instruction",
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
        self._follow_grasp_client = node.create_client(
            FollowGraspControl,
            follow_grasp_service_name,
        )
        self._joycon_client = node.create_client(
            JoyconControl,
            joycon_service_name,
        )
        self._embodied_client = node.create_client(
            EmbodiedInstruction,
            embodied_service_name,
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
            ok = ok and self._follow_grasp_client.wait_for_service(timeout_sec=timeout_sec)
            ok = ok and self._joycon_client.wait_for_service(timeout_sec=timeout_sec)
            ok = ok and self._embodied_client.wait_for_service(timeout_sec=timeout_sec)
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

    def joycon_status(self, *, timeout_sec: float = 5.0) -> SimpleServiceCallResult:
        if not self._joycon_client.wait_for_service(timeout_sec=timeout_sec):
            return SimpleServiceCallResult(False, "joycon service is not ready")
        request = JoyconControl.Request()
        request.command = "status"
        future = self._joycon_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return SimpleServiceCallResult(bool(response.success), str(response.message))

    def embodied_health(self, *, timeout_sec: float = 5.0) -> SimpleServiceCallResult:
        if not self._embodied_client.wait_for_service(timeout_sec=timeout_sec):
            return SimpleServiceCallResult(False, "embodied service is not ready")
        request = EmbodiedInstruction.Request()
        request.command = "health"
        request.stream = False
        future = self._embodied_client.call_async(request)
        response = self._wait_for_future(future, timeout_sec=timeout_sec)
        return SimpleServiceCallResult(bool(response.success), str(response.message))

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
