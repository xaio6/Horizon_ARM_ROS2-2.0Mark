from __future__ import annotations

import json
import math
import os
import sys
import types
from pathlib import Path
from typing import Any, Callable, List, Sequence

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.executors import Executor
from rclpy.duration import Duration
from trajectory_msgs.msg import JointTrajectoryPoint


DEFAULT_JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]


def prepare_sdk_import(sdk_root: str) -> None:
    _install_optional_dependency_stubs()
    sdk_root = (sdk_root or "").strip()
    if not sdk_root:
        return
    sdk_path = Path(sdk_root).expanduser().resolve()
    if not sdk_path.is_dir():
        return
    candidate_paths = [
        sdk_path,
        sdk_path / "Embodied_SDK",
        sdk_path / "Embodied_SDK" / "Horizon_Core",
    ]
    if sdk_path.name == "Embodied_SDK":
        candidate_paths.append(sdk_path.parent)
        candidate_paths.append(sdk_path / "Horizon_Core")
    for candidate in candidate_paths:
        if candidate.is_dir():
            candidate_str = str(candidate)
            if candidate_str not in sys.path:
                sys.path.insert(0, candidate_str)
    config_dir = sdk_path / "config"
    aisdk_config = config_dir / "aisdk_config.yaml"
    if config_dir.is_dir():
        os.environ.setdefault("HORIZONARM_CONFIG_DIR", str(config_dir))
    if aisdk_config.is_file():
        os.environ.setdefault("AISDK_CONFIG_PATH", str(aisdk_config))


def _install_optional_dependency_stubs() -> None:
    if "dotenv" not in sys.modules:
        dotenv_module = types.ModuleType("dotenv")

        def _load_dotenv(*args, **kwargs):
            return False

        dotenv_module.load_dotenv = _load_dotenv
        sys.modules["dotenv"] = dotenv_module


def resolve_preset_config_path(explicit_path: str) -> str:
    config_path = (explicit_path or "").strip()
    if config_path:
        return config_path
    config_dir = (os.environ.get("HORIZONARM_CONFIG_DIR") or "").strip()
    if config_dir:
        return str(Path(config_dir) / "preset_actions.json")
    return ""


def load_preset_config(explicit_path: str = "", *, required: bool = True) -> dict[str, Any]:
    config_path = resolve_preset_config_path(explicit_path)
    if not config_path:
        if required:
            raise RuntimeError(
                "preset_config_path is empty and HORIZONARM_CONFIG_DIR is not set"
            )
        return {}
    with open(config_path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def ensure_joint_vector(values: Sequence[float], expected_len: int = 6) -> List[float]:
    if len(values) < expected_len:
        raise ValueError(f"expected {expected_len} joint values, got {len(values)}")
    return [float(value) for value in values[:expected_len]]


def ensure_joint_points(values: Sequence[Any], expected_len: int = 6) -> List[List[float]]:
    if not values:
        raise ValueError("joint command is empty")
    first = values[0]
    if isinstance(first, (list, tuple)):
        return [
            ensure_joint_vector(point, expected_len=expected_len)
            for point in values
        ]
    return [ensure_joint_vector(values, expected_len=expected_len)]


def build_trajectory_goal_deg(
    joint_names: Sequence[str],
    points_deg: Sequence[Sequence[float]],
    duration_sec: float,
) -> FollowJointTrajectory.Goal:
    return build_trajectory_goal_rad(
        joint_names,
        [
            [math.radians(float(value)) for value in point]
            for point in ensure_joint_points(points_deg, expected_len=len(joint_names))
        ],
        duration_sec,
    )


def build_trajectory_goal_rad(
    joint_names: Sequence[str],
    points_rad: Sequence[Sequence[float]],
    duration_sec: float,
) -> FollowJointTrajectory.Goal:
    normalized_points = ensure_joint_points(
        points_rad,
        expected_len=len(joint_names),
    )
    duration_sec = max(0.05, float(duration_sec))
    step_duration = duration_sec / max(1, len(normalized_points))

    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = list(joint_names)
    for index, point_rad in enumerate(normalized_points, start=1):
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in point_rad]
        point.time_from_start = Duration(seconds=step_duration * index).to_msg()
        goal.trajectory.points.append(point)
    return goal


def parse_instruction_text(
    instruction: str,
    *,
    preset_config: dict[str, Any] | None = None,
) -> dict[str, Any]:
    preset_config = preset_config or {}
    text = (instruction or "").strip()
    if not text:
        raise ValueError("instruction is empty")

    if text.startswith("{"):
        payload = json.loads(text)
        if not isinstance(payload, dict):
            raise ValueError("instruction JSON must be an object")
        if "command" not in payload:
            raise ValueError("instruction JSON must contain a command field")
        return payload

    lower = text.lower()
    if lower in ("enable", "disable", "estop", "emergency_stop"):
        return {"command": lower}

    if text in preset_config:
        return {"command": "preset", "name": text}

    if lower.startswith("preset:"):
        return {"command": "preset", "name": text.split(":", 1)[1].strip()}

    if lower.startswith("set_do:"):
        payload = text.split(":", 1)[1].strip()
        if "=" in payload:
            channel_text, state_text = payload.split("=", 1)
        elif "," in payload:
            channel_text, state_text = payload.split(",", 1)
        else:
            raise ValueError("set_do instruction must look like set_do:0=1")
        return {
            "command": "set_digital_output",
            "channel": int(channel_text.strip()),
            "state": _parse_bool_text(state_text.strip()),
        }

    if lower in ("open_gripper", "gripper_open"):
        return {"command": "set_gripper_state", "open": True}

    if lower in ("close_gripper", "gripper_close", "clamp_gripper"):
        return {"command": "set_gripper_state", "open": False}

    if lower.startswith("gripper:"):
        payload = text.split(":", 1)[1].strip()
        normalized = payload.lower()
        if normalized in ("open", "opened"):
            return {"command": "set_gripper_state", "open": True}
        if normalized in ("close", "closed", "clamp"):
            return {"command": "set_gripper_state", "open": False}
        if "=" in payload:
            state_text, current_text = payload.split("=", 1)
            normalized = state_text.strip().lower()
            if normalized not in ("open", "close", "clamp"):
                raise ValueError("gripper instruction must look like gripper:open=1200")
            return {
                "command": "set_gripper_state",
                "open": normalized == "open",
                "current_ma": int(current_text.strip()),
            }
        raise ValueError("gripper instruction must look like gripper:open or gripper:close")

    raise ValueError(
        "unsupported instruction; use enable/disable/estop, "
        "preset:<name>, set_do:<channel>=<0|1>, gripper:open|close, or JSON command"
    )


def _parse_bool_text(value: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ("1", "true", "on", "high", "yes", "y"):
        return True
    if normalized in ("0", "false", "off", "low", "no", "n"):
        return False
    raise ValueError(f"cannot parse boolean value: {value}")


def spin_node_until_shutdown(
    node,
    spin: Callable[[], None],
    *,
    executor: Executor | None = None,
) -> None:
    try:
        spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
