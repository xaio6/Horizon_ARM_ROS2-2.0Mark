#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any, List

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


class ExecutePresetAction(Node):
    def __init__(self) -> None:
        super().__init__("horizon_arm_execute_preset_action")

        self.declare_parameter(
            "joint_names",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        )
        self.declare_parameter("action_name", "/horizon_arm_controller/follow_joint_trajectory")
        self.declare_parameter("preset_name", "home_position")
        self.declare_parameter("preset_config_path", "")

        self._joint_names = list(self.get_parameter("joint_names").value)
        self._preset_name = str(self.get_parameter("preset_name").value)
        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            str(self.get_parameter("action_name").value),
        )
        self._preset_config = self._load_presets(str(self.get_parameter("preset_config_path").value))

        if self._preset_name not in self._preset_config:
            raise RuntimeError(f"Unknown preset_name: {self._preset_name}")

        self._timer = self.create_timer(0.5, self._send_once)
        self._sent = False

    def _load_presets(self, explicit_path: str) -> dict[str, Any]:
        config_path = explicit_path.strip()
        if not config_path:
            config_dir = os.environ.get("HORIZONARM_CONFIG_DIR", "").strip()
            if config_dir:
                config_path = str(Path(config_dir) / "preset_actions.json")
        if not config_path:
            raise RuntimeError("preset_config_path is empty and HORIZONARM_CONFIG_DIR is not set")
        with open(config_path, "r", encoding="utf-8") as handle:
            return json.load(handle)

    def _send_once(self) -> None:
        if self._sent:
            return
        if not self._client.server_is_ready():
            self.get_logger().info("Waiting for follow_joint_trajectory action server...")
            return

        preset = self._preset_config[self._preset_name]
        joints = preset.get("joints")
        duration = float(preset.get("duration", 2.0))
        if joints is None:
            raise RuntimeError(f"Preset {self._preset_name} has no joints field")

        points_deg: List[List[float]]
        if joints and isinstance(joints[0], list):
            points_deg = [[float(value) for value in row[:6]] for row in joints]
        else:
            points_deg = [[float(value) for value in joints[:6]]]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._joint_names

        step_duration = duration / max(1, len(points_deg))
        for index, point_deg in enumerate(points_deg, start=1):
            point = JointTrajectoryPoint()
            point.positions = [math.radians(value) for value in point_deg]
            point.time_from_start = Duration(seconds=step_duration * index).to_msg()
            goal.trajectory.points.append(point)

        self.get_logger().info(
            f"Executing preset {self._preset_name}: "
            + "; ".join(
                "[" + ", ".join(f"J{i + 1}={value:+.1f}" for i, value in enumerate(point)) + "]"
                for point in points_deg
            )
        )
        self._sent = True
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Preset action goal was rejected.")
            self.destroy_node()
            rclpy.shutdown()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info(f"Preset {self._preset_name} executed successfully.")
        else:
            self.get_logger().error(
                f"Preset {self._preset_name} failed: error_code={result.error_code}, "
                f"error_string={result.error_string}"
            )
        self.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExecutePresetAction()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
