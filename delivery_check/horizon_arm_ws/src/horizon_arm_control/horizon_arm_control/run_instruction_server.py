from __future__ import annotations

import time
from typing import Any

from control_msgs.action import FollowJointTrajectory
from horizon_arm_interfaces.action import RunInstruction
from horizon_arm_interfaces.srv import SetDigitalOutput, SetGripperState
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from .common import (
    DEFAULT_JOINT_NAMES,
    build_trajectory_goal_deg,
    build_trajectory_goal_rad,
    load_preset_config,
    parse_instruction_text,
    spin_node_until_shutdown,
)


class RunInstructionServer(Node):
    """Unified high-level arm action for presets, motion, enable, estop, and DO."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_run_instruction_server")
        self._client_group = ReentrantCallbackGroup()
        self._server_group = ReentrantCallbackGroup()

        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter(
            "trajectory_action_name",
            "/horizon_arm_controller/follow_joint_trajectory",
        )
        self.declare_parameter(
            "instruction_action_name",
            "/horizon_arm/run_instruction",
        )
        self.declare_parameter("preset_config_path", "")
        self.declare_parameter("enable_service_name", "/horizon_arm/enable")
        self.declare_parameter("disable_service_name", "/horizon_arm/disable")
        self.declare_parameter("estop_service_name", "/horizon_arm/emergency_stop")
        self.declare_parameter(
            "digital_output_service_name",
            "/horizon_arm/set_digital_output",
        )
        self.declare_parameter(
            "gripper_service_name",
            "/horizon_arm/set_gripper_state",
        )
        self.declare_parameter("default_motion_timeout_sec", 20.0)
        self.declare_parameter("default_service_timeout_sec", 5.0)
        self.declare_parameter("default_gripper_current_ma", 1200)

        self._joint_names = list(self.get_parameter("joint_names").value)
        self._preset_config_path = str(self.get_parameter("preset_config_path").value)
        self._preset_config = load_preset_config(
            self._preset_config_path,
            required=False,
        )
        self._motion_timeout_sec = float(
            self.get_parameter("default_motion_timeout_sec").value
        )
        self._service_timeout_sec = float(
            self.get_parameter("default_service_timeout_sec").value
        )

        self._motion_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("trajectory_action_name").value),
            callback_group=self._client_group,
        )
        self._enable_client = self.create_client(
            Trigger,
            str(self.get_parameter("enable_service_name").value),
            callback_group=self._client_group,
        )
        self._disable_client = self.create_client(
            Trigger,
            str(self.get_parameter("disable_service_name").value),
            callback_group=self._client_group,
        )
        self._estop_client = self.create_client(
            Trigger,
            str(self.get_parameter("estop_service_name").value),
            callback_group=self._client_group,
        )
        self._digital_output_client = self.create_client(
            SetDigitalOutput,
            str(self.get_parameter("digital_output_service_name").value),
            callback_group=self._client_group,
        )
        self._gripper_client = self.create_client(
            SetGripperState,
            str(self.get_parameter("gripper_service_name").value),
            callback_group=self._client_group,
        )
        self._server = ActionServer(
            self,
            RunInstruction,
            str(self.get_parameter("instruction_action_name").value),
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._server_group,
        )

        self.get_logger().info(
            "RunInstruction server ready on "
            + str(self.get_parameter("instruction_action_name").value)
        )

    def _goal_callback(self, goal_request) -> GoalResponse:
        if not str(goal_request.instruction).strip():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        instruction = str(goal_handle.request.instruction)
        result = RunInstruction.Result()
        try:
            self._publish_feedback(goal_handle, "parse", 0.05, instruction)
            command = parse_instruction_text(
                instruction,
                preset_config=self._preset_config,
            )

            command_name = str(command.get("command", "")).strip().lower()
            if command_name == "enable":
                outcome = self._call_trigger(self._enable_client)
            elif command_name == "disable":
                outcome = self._call_trigger(self._disable_client)
            elif command_name in ("estop", "emergency_stop"):
                outcome = self._call_trigger(self._estop_client)
            elif command_name == "set_digital_output":
                outcome = self._set_digital_output(command)
            elif command_name == "set_gripper_state":
                outcome = self._set_gripper_state(command)
            elif command_name == "preset":
                outcome = self._execute_preset(goal_handle, command)
            elif command_name == "move_joints_deg":
                outcome = self._execute_motion_deg(goal_handle, command)
            elif command_name == "move_joints_rad":
                outcome = self._execute_motion_rad(goal_handle, command)
            else:
                raise ValueError(f"unsupported command: {command_name}")

            result.success = bool(outcome["success"])
            result.message = str(outcome["message"])
            if result.success:
                self._publish_feedback(goal_handle, "done", 1.0, result.message)
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        except Exception as exc:
            result.success = False
            result.message = str(exc)
            goal_handle.abort()
            return result

    def _execute_preset(self, goal_handle, command: dict[str, Any]) -> dict[str, Any]:
        preset_name = str(command.get("name", "")).strip()
        if not preset_name:
            raise ValueError("preset command is missing a name")
        if preset_name not in self._preset_config:
            raise KeyError(f"unknown preset: {preset_name}")
        preset = self._preset_config[preset_name]
        joints = preset.get("joints")
        if joints is None:
            raise ValueError(f"preset {preset_name} has no joints field")
        duration = float(preset.get("duration", 2.0))
        self._publish_feedback(goal_handle, "preset", 0.25, preset_name)
        goal = build_trajectory_goal_deg(self._joint_names, joints, duration)
        return self._execute_motion_goal(goal_handle, goal, label=f"preset {preset_name}")

    def _execute_motion_deg(self, goal_handle, command: dict[str, Any]) -> dict[str, Any]:
        joints = command.get("joints")
        if joints is None:
            raise ValueError("move_joints_deg command is missing joints")
        duration = float(command.get("duration", 2.0))
        self._publish_feedback(goal_handle, "motion", 0.25, "move_joints_deg")
        goal = build_trajectory_goal_deg(self._joint_names, joints, duration)
        return self._execute_motion_goal(goal_handle, goal, label="move_joints_deg")

    def _execute_motion_rad(self, goal_handle, command: dict[str, Any]) -> dict[str, Any]:
        joints = command.get("joints")
        if joints is None:
            raise ValueError("move_joints_rad command is missing joints")
        duration = float(command.get("duration", 2.0))
        self._publish_feedback(goal_handle, "motion", 0.25, "move_joints_rad")
        goal = build_trajectory_goal_rad(self._joint_names, joints, duration)
        return self._execute_motion_goal(goal_handle, goal, label="move_joints_rad")

    def _execute_motion_goal(
        self,
        goal_handle,
        goal: FollowJointTrajectory.Goal,
        *,
        label: str,
    ) -> dict[str, Any]:
        if not self._motion_client.wait_for_server(timeout_sec=self._motion_timeout_sec):
            return {
                "success": False,
                "message": "follow_joint_trajectory action server is not ready",
            }

        self._publish_feedback(goal_handle, "send_goal", 0.45, label)
        send_future = self._motion_client.send_goal_async(goal)
        sent_goal_handle = self._wait_for_future(
            send_future,
            timeout_sec=self._motion_timeout_sec,
        )
        if not sent_goal_handle.accepted:
            return {
                "success": False,
                "message": f"{label} goal was rejected",
            }

        self._publish_feedback(goal_handle, "execute", 0.75, label)
        result_future = sent_goal_handle.get_result_async()
        wrapped_result = self._wait_for_future(
            result_future,
            timeout_sec=self._motion_timeout_sec,
        )
        result = wrapped_result.result
        success = result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        return {
            "success": success,
            "message": (
                f"{label} completed successfully"
                if success
                else (
                    f"{label} failed: error_code={result.error_code}, "
                    f"error_string={result.error_string}"
                )
            ),
        }

    def _call_trigger(self, client) -> dict[str, Any]:
        if not client.wait_for_service(timeout_sec=self._service_timeout_sec):
            return {"success": False, "message": "service is not ready"}
        future = client.call_async(Trigger.Request())
        response = self._wait_for_future(
            future,
            timeout_sec=self._service_timeout_sec,
        )
        return {
            "success": bool(response.success),
            "message": str(response.message),
        }

    def _set_digital_output(self, command: dict[str, Any]) -> dict[str, Any]:
        if not self._digital_output_client.wait_for_service(
            timeout_sec=self._service_timeout_sec
        ):
            return {"success": False, "message": "digital output service is not ready"}
        request = SetDigitalOutput.Request()
        request.channel = int(command["channel"])
        request.state = bool(command["state"])
        future = self._digital_output_client.call_async(request)
        response = self._wait_for_future(
            future,
            timeout_sec=self._service_timeout_sec,
        )
        return {
            "success": bool(response.success),
            "message": str(response.message),
        }

    def _set_gripper_state(self, command: dict[str, Any]) -> dict[str, Any]:
        if not self._gripper_client.wait_for_service(
            timeout_sec=self._service_timeout_sec
        ):
            return {"success": False, "message": "gripper service is not ready"}
        request = SetGripperState.Request()
        request.open = bool(command["open"])
        current_ma = int(
            command.get(
                "current_ma",
                int(self.get_parameter("default_gripper_current_ma").value),
            )
        )
        request.current_ma = max(0, current_ma)
        future = self._gripper_client.call_async(request)
        response = self._wait_for_future(
            future,
            timeout_sec=self._service_timeout_sec,
        )
        return {
            "success": bool(response.success),
            "message": str(response.message),
        }

    def _wait_for_future(self, future, *, timeout_sec: float):
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.01)
        raise TimeoutError("ROS2 call timed out")

    def _publish_feedback(
        self,
        goal_handle,
        stage: str,
        progress: float,
        detail: str,
    ) -> None:
        feedback = RunInstruction.Feedback()
        feedback.stage = str(stage)
        feedback.progress = float(progress)
        feedback.detail = str(detail)
        goal_handle.publish_feedback(feedback)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RunInstructionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_node_until_shutdown(node, executor.spin, executor=executor)


if __name__ == "__main__":
    main()
