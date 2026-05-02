from __future__ import annotations

import dataclasses
import datetime as dt
import html
import importlib
import json
import os
import time
from pathlib import Path
from typing import Any, Callable, List, Optional

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


class SystemCheckNode(Node):
    """Unified ROS2 runtime and Linux SDK self-check."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_system_check")

        self.declare_parameter("sdk_root", "")
        self.declare_parameter("report_dir", "")
        self.declare_parameter("status_timeout_sec", 4.0)
        self.declare_parameter("joint_state_timeout_sec", 4.0)
        self.declare_parameter("service_timeout_sec", 2.0)
        self.declare_parameter("action_timeout_sec", 4.0)
        self.declare_parameter("camera_id", 0)
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
        self._follow_grasp_client = self.create_client(
            FollowGraspControl, "/horizon_arm/follow_grasp_control"
        )
        self._joycon_client = self.create_client(
            JoyconControl, "/horizon_arm/joycon_control"
        )
        self._embodied_client = self.create_client(
            EmbodiedInstruction, "/horizon_arm/embodied_instruction"
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

    def run(self) -> int:
        prepare_sdk_import(str(self.get_parameter("sdk_root").value))
        self._check_ros_runtime()
        self._check_sdk_surface()
        self._check_optional_live_tests()
        self._write_reports()
        self._print_summary()
        return 0 if not any(item.status == "FAIL" for item in self._results) else 1

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
        self._check_service(self._do_client, "/horizon_arm/set_digital_output")
        self._check_service(self._gripper_client, "/horizon_arm/set_gripper_state")
        self._check_service(self._visual_grasp_client, "/horizon_arm/visual_grasp")
        self._check_service(
            self._follow_grasp_client, "/horizon_arm/follow_grasp_control"
        )
        self._check_service(self._joycon_client, "/horizon_arm/joycon_control")
        self._check_service(
            self._embodied_client, "/horizon_arm/embodied_instruction"
        )
        self._check_action(
            self._traj_client, "/horizon_arm_controller/follow_joint_trajectory"
        )
        self._check_action(self._instruction_client, "/horizon_arm/run_instruction")

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
                        "Visual grasp and follow grasp wrappers are exposed via "
                        "/horizon_arm/visual_grasp and /horizon_arm/follow_grasp_control."
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
                        "Joycon wrapper is exposed via /horizon_arm/joycon_control "
                        "for status/connect/start/stop commands."
                    ),
                    recommendation=(
                        "Use the status command by default and connect/start only when the controller is present."
                    ),
                ),
                CheckResult(
                    category="capability",
                    name="Embodied AI ROS2 wrapper",
                    status="PASS",
                    detail=(
                        "Embodied AI wrapper is exposed via /horizon_arm/embodied_instruction "
                        "with a safe health command."
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
        else:
            self._skip("live", "RunInstruction smoke test", "Disabled by parameter.")

        if bool(self.get_parameter("preset_test_enabled").value):
            preset_name = str(self.get_parameter("preset_name").value)
            self._test_instruction_smoke(
                f"preset:{preset_name}",
                f"Preset motion test ({preset_name})",
            )
        else:
            self._skip("live", "Preset motion test", "Disabled by parameter.")

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
        else:
            self._skip("live", "Single-axis motion test", "Disabled by parameter.")

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
        else:
            self._skip("live", "Multi-axis motion test", "Disabled by parameter.")

        if bool(self.get_parameter("digital_output_test_enabled").value):
            self._test_digital_output()
        else:
            self._skip("live", "Digital output live test", "Disabled by parameter.")

        if bool(self.get_parameter("gripper_test_enabled").value):
            self._test_gripper()
        else:
            self._skip("live", "Gripper close test", "Disabled by parameter.")
            self._skip("live", "Gripper open test", "Disabled by parameter.")

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

    def _check_service(self, client, service_name: str) -> None:
        timeout_sec = float(self.get_parameter("service_timeout_sec").value)
        ready = client.wait_for_service(timeout_sec=timeout_sec)
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

    def _call_simple_service(
        self,
        client,
        request,
        *,
        result_name: str,
        timeout_sec: float,
        not_ready_detail: str = "Service is not ready.",
    ) -> None:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self._results.append(
                CheckResult(
                    category="live",
                    name=result_name,
                    status="FAIL",
                    detail=not_ready_detail,
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
                )
            )
        except Exception as exc:
            self._results.append(
                CheckResult(
                    category="live",
                    name=result_name,
                    status="FAIL",
                    detail=f"Service call failed: {exc}",
                )
            )

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
        }

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

    def _print_summary(self) -> None:
        counts = self._counts()
        print("\n=== Horizon Arm System Check Summary ===")
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

