from __future__ import annotations

import importlib
import json
import math
import time
from pathlib import Path
from typing import Any

from control_msgs.action import FollowJointTrajectory
from horizon_arm_interfaces.action import TeachingProgram
from horizon_arm_interfaces.srv import TeachJog
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .common import (
    DEFAULT_JOINT_NAMES,
    build_trajectory_goal_deg,
    ensure_joint_vector,
    prepare_sdk_import,
    spin_node_until_shutdown,
)


class TeachingServer(Node):
    """ROS2 teaching-pendant bridge for jog commands and teaching programs."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_teaching_server")
        self._client_group = ReentrantCallbackGroup()
        self._server_group = ReentrantCallbackGroup()

        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("sdk_root", "")
        self.declare_parameter(
            "trajectory_action_name",
            "/horizon_arm_controller/follow_joint_trajectory",
        )
        self.declare_parameter("jog_service_name", "/horizon_arm/teach_jog")
        self.declare_parameter(
            "program_action_name",
            "/horizon_arm/teaching_program",
        )
        self.declare_parameter("default_duration_sec", 2.0)
        self.declare_parameter("motion_timeout_sec", 30.0)

        self._joint_names = list(self.get_parameter("joint_names").value)
        self._latest_joint_state: dict[str, float] = {}
        self._motion_timeout_sec = float(self.get_parameter("motion_timeout_sec").value)
        self._kinematics_bundle: dict[str, Any] | None = None
        self._last_interpolation_config: dict[str, Any] = {}

        self._motion_client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("trajectory_action_name").value),
            callback_group=self._client_group,
        )
        self._joint_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            10,
        )
        self._jog_service = self.create_service(
            TeachJog,
            str(self.get_parameter("jog_service_name").value),
            self._on_teach_jog,
            callback_group=self._server_group,
        )
        self._program_server = ActionServer(
            self,
            TeachingProgram,
            str(self.get_parameter("program_action_name").value),
            execute_callback=self._execute_program,
            goal_callback=self._program_goal_callback,
            cancel_callback=self._program_cancel_callback,
            callback_group=self._server_group,
        )

        self.get_logger().info(
            "Teach jog service ready on "
            + str(self.get_parameter("jog_service_name").value)
        )
        self.get_logger().info(
            "Teaching program action ready on "
            + str(self.get_parameter("program_action_name").value)
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            self._latest_joint_state[str(name)] = float(position)

    def _on_teach_jog(self, request, response):
        command = str(request.command).strip().lower() or "status"
        try:
            if command == "status":
                response.success = True
                response.message = "teaching server status queried"
                response.target_joint_angles = self._current_joint_angles_deg()
                response.detail_json = json.dumps(
                    {
                        "joint_names": self._joint_names,
                        "has_joint_state": bool(self._latest_joint_state),
                        "interpolation_type": str(request.interpolation_type),
                    },
                    ensure_ascii=False,
                )
                return response

            if command == "joint_jog":
                target = self._current_joint_angles_deg()
                joint_index = int(request.joint_index)
                if joint_index < 1 or joint_index > len(target):
                    raise ValueError("joint_index is 1-based and must be 1..6")
                target[joint_index - 1] += float(request.delta)
                return self._finish_jog_response(
                    request,
                    response,
                    target,
                    label=f"joint_jog J{joint_index}",
                )

            if command == "joint_move":
                target = ensure_joint_vector(
                    list(request.joint_angles),
                    expected_len=len(self._joint_names),
                )
                return self._finish_jog_response(
                    request,
                    response,
                    target,
                    label="joint_move",
                )

            if command in (
                "base_translate",
                "tool_translate",
                "base_rotate",
                "tool_rotate",
                "cartesian_move",
            ):
                plan = self._plan_cartesian_command(command, request)
                response.target_joint_angles = [
                    float(value) for value in plan["target_joint_angles"]
                ]
                response.detail_json = json.dumps(plan, ensure_ascii=False)
                if bool(request.dry_run):
                    response.success = True
                    response.message = f"{command} planned"
                    return response
                duration = self._duration_from_request(request, target_deg=plan["target_joint_angles"])
                outcome = self._execute_joint_points(
                    [plan["target_joint_angles"]],
                    duration_sec=duration,
                )
                response.success = bool(outcome["success"])
                response.message = str(outcome["message"])
                return response

            if command == "set_interpolation":
                self._last_interpolation_config = {
                    "interpolation_type": str(request.interpolation_type),
                    "max_speed": float(request.max_speed),
                    "acceleration": float(request.acceleration),
                    "deceleration": float(request.deceleration),
                    "linear_velocity": float(request.linear_velocity),
                    "angular_velocity": float(request.angular_velocity),
                    "linear_acceleration": float(request.linear_acceleration),
                    "angular_acceleration": float(request.angular_acceleration),
                    "joint_max_velocities": list(request.joint_max_velocities),
                    "joint_max_accelerations": list(request.joint_max_accelerations),
                }
                response.success = True
                response.message = "interpolation config updated"
                response.target_joint_angles = self._current_joint_angles_deg()
                response.detail_json = json.dumps(
                    self._last_interpolation_config,
                    ensure_ascii=False,
                )
                return response

            if command == "stop":
                response.success = True
                response.message = "stop accepted; no active cancelable cartesian executor is running"
                response.target_joint_angles = self._current_joint_angles_deg()
                response.detail_json = json.dumps(
                    {"command": "stop"},
                    ensure_ascii=False,
                )
                return response

            response.success = False
            response.message = (
                "unsupported command; use status/joint_jog/joint_move/"
                "base_translate/tool_translate/base_rotate/tool_rotate/"
                "cartesian_move/set_interpolation/stop"
            )
            response.target_joint_angles = self._current_joint_angles_deg()
            response.detail_json = "{}"
        except Exception as exc:
            response.success = False
            response.message = f"teach jog failed: {exc}"
            response.target_joint_angles = []
            response.detail_json = "{}"
        return response

    def _finish_jog_response(self, request, response, target_deg, *, label: str):
        response.target_joint_angles = [float(value) for value in target_deg]
        response.detail_json = json.dumps(
            {
                "label": label,
                "interpolation_type": str(request.interpolation_type) or "joint",
                "max_speed": float(request.max_speed),
                "acceleration": float(request.acceleration),
                "deceleration": float(request.deceleration),
                "dry_run": bool(request.dry_run),
            },
            ensure_ascii=False,
        )
        if bool(request.dry_run):
            response.success = True
            response.message = f"{label} planned"
            return response

        duration = self._duration_from_request(request)
        outcome = self._execute_joint_points([target_deg], duration_sec=duration)
        response.success = bool(outcome["success"])
        response.message = str(outcome["message"])
        return response

    def _program_goal_callback(self, goal_request) -> GoalResponse:
        command = str(goal_request.command).strip().lower()
        if command not in ("run", "load", "save", "status", "validate"):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _program_cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        return CancelResponse.ACCEPT

    def _execute_program(self, goal_handle):
        command = str(goal_handle.request.command).strip().lower() or "run"
        result = TeachingProgram.Result()
        try:
            if command == "status":
                self._publish_program_feedback(goal_handle, 0, 0, "status", "ready")
                result.success = True
                result.message = "teaching program server ready"
                result.result_json = "{}"
                goal_handle.succeed()
                return result

            if command == "save":
                payload = self._load_program_payload(goal_handle.request)
                program_path = self._resolve_program_path(goal_handle.request)
                program_path.parent.mkdir(parents=True, exist_ok=True)
                program_path.write_text(
                    json.dumps(payload, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
                result.success = True
                result.message = f"program saved: {program_path}"
                result.result_json = json.dumps(payload, ensure_ascii=False)
                goal_handle.succeed()
                return result

            payload = self._load_program_payload(goal_handle.request)
            points = self._extract_program_joint_points(payload)
            total = len(points)
            self._publish_program_feedback(
                goal_handle,
                0,
                total,
                "validate",
                f"{total} teaching points",
            )

            if command in ("load", "validate") or bool(goal_handle.request.dry_run):
                result.success = True
                result.message = f"program validated: {total} joint points"
                result.result_json = json.dumps(
                    {"total": total, "points": points},
                    ensure_ascii=False,
                )
                goal_handle.succeed()
                return result

            duration = float(self.get_parameter("default_duration_sec").value) * max(
                1,
                total,
            )
            self._publish_program_feedback(goal_handle, 0, total, "execute", "send goal")
            outcome = self._execute_joint_points(points, duration_sec=duration)
            result.success = bool(outcome["success"])
            result.message = str(outcome["message"])
            result.result_json = json.dumps(
                {"total": total, "points": points},
                ensure_ascii=False,
            )
            if result.success:
                self._publish_program_feedback(
                    goal_handle,
                    total,
                    total,
                    "done",
                    result.message,
                )
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        except Exception as exc:
            result.success = False
            result.message = str(exc)
            result.result_json = "{}"
            goal_handle.abort()
            return result

    def _load_program_payload(self, request) -> dict[str, Any]:
        if str(request.program_json).strip():
            payload = json.loads(str(request.program_json))
        else:
            program_path = self._resolve_program_path(request)
            payload = json.loads(program_path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("teaching program JSON must be an object")
        return payload

    def _resolve_program_path(self, request) -> Path:
        if str(request.program_path).strip():
            return Path(str(request.program_path)).expanduser().resolve()
        if str(request.program_name).strip():
            return Path(str(request.program_name)).expanduser().resolve()
        raise ValueError("program_path or program_name is required")

    def _extract_program_joint_points(self, payload: dict[str, Any]) -> list[list[float]]:
        raw_points = payload.get("points", payload.get("teaching_points", []))
        if not isinstance(raw_points, list) or len(raw_points) == 0:
            raise ValueError("teaching program must contain points")
        points = []
        for item in raw_points:
            joints = item.get("joint_angles") if isinstance(item, dict) else item
            points.append(ensure_joint_vector(joints, expected_len=len(self._joint_names)))
        return points

    def _current_joint_angles_deg(self) -> list[float]:
        if not self._latest_joint_state:
            return [0.0 for _ in self._joint_names]
        return [
            math.degrees(float(self._latest_joint_state.get(name, 0.0)))
            for name in self._joint_names
        ]

    def _duration_from_request(self, request, *, target_deg: list[float] | None = None) -> float:
        max_speed = float(request.max_speed)
        if max_speed > 0:
            return max(0.2, 180.0 / max_speed)
        if target_deg is not None:
            current_deg = self._current_joint_angles_deg()
            max_delta = max(
                abs(float(target_deg[index]) - float(current_deg[index]))
                for index in range(len(self._joint_names))
            )
            if max_delta > 0.0:
                joint_velocities = [float(v) for v in list(request.joint_max_velocities)]
                usable_max_speed = max([value for value in joint_velocities if value > 0.0], default=90.0)
                return max(0.2, max_delta / max(1.0, usable_max_speed) * 1.5)
        linear_velocity = float(request.linear_velocity)
        if linear_velocity > 0.0:
            return max(0.2, 100.0 / linear_velocity)
        return float(self.get_parameter("default_duration_sec").value)

    def _plan_cartesian_command(self, command: str, request) -> dict[str, Any]:
        current_deg = self._current_joint_angles_deg()
        target_transform, current_pose, target_pose = self._build_target_transform_from_request(
            command,
            request,
            current_deg,
        )
        target_joint_angles = self._solve_joint_target_from_transform(
            target_transform,
            current_deg,
        )
        return {
            "command": command,
            "frame": str(request.frame),
            "axis": str(request.axis),
            "delta": float(request.delta),
            "current_pose": current_pose,
            "target_pose": target_pose,
            "target_joint_angles": target_joint_angles,
            "interpolation_type": str(request.interpolation_type) or "cartesian",
            "linear_velocity": float(request.linear_velocity),
            "angular_velocity": float(request.angular_velocity),
            "linear_acceleration": float(request.linear_acceleration),
            "angular_acceleration": float(request.angular_acceleration),
            "joint_max_velocities": list(request.joint_max_velocities),
            "joint_max_accelerations": list(request.joint_max_accelerations),
        }

    def _build_target_transform_from_request(
        self,
        command: str,
        request,
        current_deg: list[float],
    ) -> tuple[list[list[float]], dict[str, Any], dict[str, Any]]:
        transform = self._forward_kinematics(current_deg)
        current_position = [transform[0][3], transform[1][3], transform[2][3]]
        current_rotation = [
            [transform[row][col] for col in range(3)]
            for row in range(3)
        ]
        current_orientation = self._rotation_matrix_to_euler(current_rotation)

        target_position = list(current_position)
        target_rotation = [row[:] for row in current_rotation]
        delta = float(request.delta)
        axis = str(request.axis).strip().lower()
        frame = str(request.frame).strip().lower() or "base"

        if command == "base_translate":
            axis_vector = self._translation_axis_vector(axis)
            target_position = [
                current_position[index] + axis_vector[index] * delta
                for index in range(3)
            ]
        elif command == "tool_translate":
            axis_vector = self._translation_axis_vector(axis)
            tool_delta = self._mat_vec_mul(current_rotation, [value * delta for value in axis_vector])
            target_position = [
                current_position[index] + tool_delta[index]
                for index in range(3)
            ]
        elif command == "base_rotate":
            axis_vector = self._rotation_axis_vector(axis)
            delta_rotation = self._axis_angle_rotation(axis_vector, math.radians(delta))
            target_rotation = self._mat3_mul(delta_rotation, current_rotation)
        elif command == "tool_rotate":
            axis_vector = self._rotation_axis_vector(axis)
            delta_rotation = self._axis_angle_rotation(axis_vector, math.radians(delta))
            target_rotation = self._mat3_mul(current_rotation, delta_rotation)
        elif command == "cartesian_move":
            position = [float(v) for v in list(request.position)]
            orientation = [float(v) for v in list(request.orientation)]
            if frame == "tool":
                if len(position) >= 3:
                    tool_delta = self._mat_vec_mul(current_rotation, position[:3])
                    target_position = [
                        current_position[index] + tool_delta[index]
                        for index in range(3)
                    ]
                if len(orientation) >= 3:
                    delta_rotation = self._euler_to_rotation_matrix(
                        orientation[0],
                        orientation[1],
                        orientation[2],
                    )
                    target_rotation = self._mat3_mul(current_rotation, delta_rotation)
            else:
                if len(position) >= 3:
                    target_position = position[:3]
                if len(orientation) >= 3:
                    target_rotation = self._euler_to_rotation_matrix(
                        orientation[0],
                        orientation[1],
                        orientation[2],
                    )

        target_orientation = self._rotation_matrix_to_euler(target_rotation)
        target_transform = self._build_transform_from_pose(
            target_position,
            target_orientation,
            target_rotation,
        )
        current_pose = {
            "position": [float(value) for value in current_position],
            "orientation": [float(value) for value in current_orientation],
        }
        target_pose = {
            "position": [float(value) for value in target_position],
            "orientation": [float(value) for value in target_orientation],
        }
        return target_transform, current_pose, target_pose

    def _solve_joint_target_from_transform(
        self,
        transform: list[list[float]],
        current_deg: list[float],
    ) -> list[float]:
        bundle = self._get_kinematics_bundle()
        kinematics = bundle["kinematics"]
        ik_transform = transform
        try:
            import numpy as np  # type: ignore

            ik_transform = np.array(transform, dtype=float)
        except Exception:
            ik_transform = transform
        solutions = kinematics.inverse_kinematics(ik_transform, return_all=True)
        normalized = self._normalize_ik_solutions(solutions)
        if not normalized:
            raise ValueError("cartesian IK failed: no solution")

        closest_selector = getattr(kinematics, "select_closest_solution", None)
        if callable(closest_selector):
            try:
                selected_result = closest_selector(solutions, current_deg)
                if isinstance(selected_result, dict) and "normalized" in selected_result:
                    validated = ensure_joint_vector(
                        selected_result["normalized"],
                        expected_len=len(self._joint_names),
                    )
                    self._ensure_finite_joint_vector(validated)
                    return validated
            except Exception:
                pass

        internal = bundle.get("internal")
        if internal is not None and hasattr(internal, "select_best_solution"):
            try:
                selected = internal.select_best_solution(
                    normalized,
                    reference_angles=current_deg,
                )
                validated = ensure_joint_vector(selected, expected_len=len(self._joint_names))
                self._ensure_finite_joint_vector(validated)
                return validated
            except Exception:
                pass

        best = min(
            normalized,
            key=lambda item: sum(abs(float(item[idx]) - float(current_deg[idx])) for idx in range(len(self._joint_names))),
        )
        validated = ensure_joint_vector(best, expected_len=len(self._joint_names))
        self._ensure_finite_joint_vector(validated)
        return validated

    def _ensure_finite_joint_vector(self, values: list[float]) -> None:
        if any(not math.isfinite(float(value)) for value in values):
            raise ValueError("cartesian IK returned non-finite joint angles")

    def _normalize_ik_solutions(self, solutions: Any) -> list[list[float]]:
        if solutions is None:
            return []
        if hasattr(solutions, "tolist"):
            solutions = solutions.tolist()
        if not isinstance(solutions, list):
            return []
        if solutions and isinstance(solutions[0], (int, float)):
            return [ensure_joint_vector(solutions, expected_len=len(self._joint_names))]
        normalized = []
        for item in solutions:
            try:
                candidate = ensure_joint_vector(item, expected_len=len(self._joint_names))
                if all(math.isfinite(float(value)) for value in candidate):
                    normalized.append(candidate)
            except Exception:
                continue
        return normalized

    def _get_kinematics_bundle(self) -> dict[str, Any]:
        if self._kinematics_bundle is not None:
            return self._kinematics_bundle
        sdk_root = str(self.get_parameter("sdk_root").value)
        prepare_sdk_import(sdk_root)
        factory = importlib.import_module(
            "Embodied_SDK.Horizon_Core.core.arm_core.kinematics_factory"
        )
        gateway = importlib.import_module("Embodied_SDK.Horizon_Core.gateway")
        self._kinematics_bundle = {
            "config": factory.load_kinematics_config(),
            "kinematics": factory.create_configured_kinematics(),
            "internal": gateway.get_embodied_internal_module(),
        }
        return self._kinematics_bundle

    def _forward_kinematics(self, joint_deg: list[float]) -> list[list[float]]:
        config = self._get_kinematics_bundle()["config"]
        dh = config.get("dh_parameters", {})
        d_values = [float(v) for v in dh.get("d", [])]
        a_values = [float(v) for v in dh.get("a", [])]
        alpha_values = [math.radians(float(v)) for v in dh.get("alpha_deg", [])]
        offsets = [float(v) for v in config.get("joint_offsets", [0.0] * len(joint_deg))]
        use_offset = bool(config.get("enable_offset", True))

        transform = self._identity4()
        for index in range(len(self._joint_names)):
            theta_deg = float(joint_deg[index]) + (offsets[index] if use_offset else 0.0)
            theta = math.radians(theta_deg)
            d = d_values[index]
            a = a_values[index]
            alpha = alpha_values[index]
            joint_transform = [
                [
                    math.cos(theta),
                    -math.sin(theta) * math.cos(alpha),
                    math.sin(theta) * math.sin(alpha),
                    a * math.cos(theta),
                ],
                [
                    math.sin(theta),
                    math.cos(theta) * math.cos(alpha),
                    -math.cos(theta) * math.sin(alpha),
                    a * math.sin(theta),
                ],
                [0.0, math.sin(alpha), math.cos(alpha), d],
                [0.0, 0.0, 0.0, 1.0],
            ]
            transform = self._mat4_mul(transform, joint_transform)
        return transform

    def _translation_axis_vector(self, axis: str) -> list[float]:
        mapping = {
            "x": [1.0, 0.0, 0.0],
            "y": [0.0, 1.0, 0.0],
            "z": [0.0, 0.0, 1.0],
        }
        if axis not in mapping:
            raise ValueError("translation axis must be x, y, or z")
        return mapping[axis]

    def _rotation_axis_vector(self, axis: str) -> list[float]:
        mapping = {
            "roll": [1.0, 0.0, 0.0],
            "pitch": [0.0, 1.0, 0.0],
            "yaw": [0.0, 0.0, 1.0],
            "x": [1.0, 0.0, 0.0],
            "y": [0.0, 1.0, 0.0],
            "z": [0.0, 0.0, 1.0],
        }
        if axis not in mapping:
            raise ValueError("rotation axis must be roll, pitch, yaw, x, y, or z")
        return mapping[axis]

    def _identity4(self) -> list[list[float]]:
        return [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def _mat4_mul(self, left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
        result = [[0.0 for _ in range(4)] for _ in range(4)]
        for row in range(4):
            for col in range(4):
                result[row][col] = sum(left[row][k] * right[k][col] for k in range(4))
        return result

    def _mat3_mul(self, left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
        result = [[0.0 for _ in range(3)] for _ in range(3)]
        for row in range(3):
            for col in range(3):
                result[row][col] = sum(left[row][k] * right[k][col] for k in range(3))
        return result

    def _mat_vec_mul(self, matrix: list[list[float]], vector: list[float]) -> list[float]:
        return [
            sum(matrix[row][col] * vector[col] for col in range(3))
            for row in range(3)
        ]

    def _axis_angle_rotation(self, axis: list[float], angle_rad: float) -> list[list[float]]:
        x, y, z = axis
        c = math.cos(angle_rad)
        s = math.sin(angle_rad)
        t = 1.0 - c
        return [
            [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
        ]

    def _compose_transform(
        self,
        position: list[float],
        rotation: list[list[float]],
    ) -> list[list[float]]:
        return [
            [rotation[0][0], rotation[0][1], rotation[0][2], float(position[0])],
            [rotation[1][0], rotation[1][1], rotation[1][2], float(position[1])],
            [rotation[2][0], rotation[2][1], rotation[2][2], float(position[2])],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def _build_transform_from_pose(
        self,
        position: list[float],
        orientation: list[float],
        fallback_rotation: list[list[float]],
    ) -> list[list[float]]:
        bundle = self._get_kinematics_bundle()
        internal = bundle.get("internal")
        if internal is not None and hasattr(internal, "_build_target_transform"):
            try:
                transform = internal._build_target_transform(
                    list(position),
                    list(orientation),
                )
                if hasattr(transform, "tolist"):
                    transform = transform.tolist()
                if isinstance(transform, list):
                    return transform
            except Exception:
                pass
        return self._compose_transform(position, fallback_rotation)

    def _euler_to_rotation_matrix(
        self,
        yaw_deg: float,
        pitch_deg: float,
        roll_deg: float,
    ) -> list[list[float]]:
        yaw = math.radians(float(yaw_deg))
        pitch = math.radians(float(pitch_deg))
        roll = math.radians(float(roll_deg))
        cz, sz = math.cos(yaw), math.sin(yaw)
        cy, sy = math.cos(pitch), math.sin(pitch)
        cx, sx = math.cos(roll), math.sin(roll)
        rz = [[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]]
        ry = [[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]]
        rx = [[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]]
        return self._mat3_mul(self._mat3_mul(rz, ry), rx)

    def _rotation_matrix_to_euler(self, rotation: list[list[float]]) -> list[float]:
        sy = math.sqrt(rotation[0][0] ** 2 + rotation[1][0] ** 2)
        singular = sy < 1e-6
        if not singular:
            roll = math.atan2(rotation[2][1], rotation[2][2])
            pitch = math.atan2(-rotation[2][0], sy)
            yaw = math.atan2(rotation[1][0], rotation[0][0])
        else:
            roll = math.atan2(-rotation[1][2], rotation[1][1])
            pitch = math.atan2(-rotation[2][0], sy)
            yaw = 0.0
        return [
            math.degrees(yaw),
            math.degrees(pitch),
            math.degrees(roll),
        ]

    def _execute_joint_points(
        self,
        points_deg: list[list[float]],
        *,
        duration_sec: float,
    ) -> dict[str, Any]:
        if not self._motion_client.wait_for_server(timeout_sec=self._motion_timeout_sec):
            return {
                "success": False,
                "message": "follow_joint_trajectory action server is not ready",
            }
        goal = build_trajectory_goal_deg(self._joint_names, points_deg, duration_sec)
        send_future = self._motion_client.send_goal_async(goal)
        sent_goal_handle = self._wait_for_future(
            send_future,
            timeout_sec=self._motion_timeout_sec,
        )
        if not sent_goal_handle.accepted:
            return {"success": False, "message": "teaching motion goal was rejected"}
        wrapped_result = self._wait_for_future(
            sent_goal_handle.get_result_async(),
            timeout_sec=max(self._motion_timeout_sec, duration_sec + 5.0),
        )
        motion_result = wrapped_result.result
        success = motion_result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
        return {
            "success": success,
            "message": (
                "teaching motion completed"
                if success
                else (
                    "teaching motion failed: "
                    f"error_code={motion_result.error_code}, "
                    f"error_string={motion_result.error_string}"
                )
            ),
        }

    def _wait_for_future(self, future, *, timeout_sec: float):
        deadline = time.monotonic() + max(0.1, float(timeout_sec))
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.01)
        raise TimeoutError("ROS2 call timed out")

    def _publish_program_feedback(
        self,
        goal_handle,
        current_index: int,
        total: int,
        stage: str,
        detail: str,
    ) -> None:
        feedback = TeachingProgram.Feedback()
        feedback.current_index = int(current_index)
        feedback.total = int(total)
        feedback.progress = 1.0 if total <= 0 else float(current_index) / float(total)
        feedback.stage = str(stage)
        feedback.detail = str(detail)
        goal_handle.publish_feedback(feedback)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TeachingServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_node_until_shutdown(node, executor.spin, executor=executor)


if __name__ == "__main__":
    main()
