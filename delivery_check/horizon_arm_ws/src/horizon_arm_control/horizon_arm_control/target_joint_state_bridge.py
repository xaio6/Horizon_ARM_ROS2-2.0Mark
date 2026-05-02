from __future__ import annotations

import math
from typing import List, Optional

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class TargetJointStateBridge(Node):
    """Convert target JointState messages into FollowJointTrajectory goals."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_target_joint_state_bridge")

        self.declare_parameter(
            "joint_names",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        )
        self.declare_parameter("target_topic", "/horizon_arm/target_joint_states")
        self.declare_parameter(
            "action_name", "/horizon_arm_controller/follow_joint_trajectory"
        )
        self.declare_parameter("send_rate_hz", 2.0)
        self.declare_parameter("trajectory_duration_sec", 1.0)
        self.declare_parameter("min_delta_rad", 0.005)
        self.declare_parameter("target_settle_time_sec", 0.0)
        self.declare_parameter("result_cooldown_sec", 0.05)
        self.declare_parameter("require_target_change_after_start", True)
        self.declare_parameter("current_state_topic", "/joint_states")
        self.declare_parameter("hold_unchanged_joints_at_current", False)

        self._joint_names = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        target_topic = (
            self.get_parameter("target_topic").get_parameter_value().string_value
        )
        action_name = (
            self.get_parameter("action_name").get_parameter_value().string_value
        )
        send_rate_hz = max(
            0.5, self.get_parameter("send_rate_hz").get_parameter_value().double_value
        )

        self._duration_sec = max(
            0.1,
            self.get_parameter("trajectory_duration_sec")
            .get_parameter_value()
            .double_value,
        )
        self._min_delta = max(
            0.0,
            self.get_parameter("min_delta_rad").get_parameter_value().double_value,
        )
        self._target_settle_time_ns = int(
            max(
                0.0,
                self.get_parameter("target_settle_time_sec")
                .get_parameter_value()
                .double_value,
            )
            * 1_000_000_000
        )
        self._result_cooldown_ns = int(
            max(
                0.0,
                self.get_parameter("result_cooldown_sec")
                .get_parameter_value()
                .double_value,
            )
            * 1_000_000_000
        )
        self._require_target_change = (
            self.get_parameter("require_target_change_after_start")
            .get_parameter_value()
            .bool_value
        )
        current_state_topic = (
            self.get_parameter("current_state_topic").get_parameter_value().string_value
        )
        self._hold_unchanged_joints = (
            self.get_parameter("hold_unchanged_joints_at_current")
            .get_parameter_value()
            .bool_value
        )
        self._first_target_positions: Optional[List[float]] = None
        self._pending_positions: Optional[List[float]] = None
        self._pending_updated_ns: Optional[int] = None
        self._active_positions: Optional[List[float]] = None
        self._last_sent_positions: Optional[List[float]] = None
        self._current_positions: Optional[List[float]] = None
        self._last_result_ns: Optional[int] = None
        self._goal_in_flight = False

        self._client = ActionClient(self, FollowJointTrajectory, action_name)
        self._subscription = self.create_subscription(
            JointState, target_topic, self._on_target_joint_state, 10
        )
        self._current_subscription = self.create_subscription(
            JointState, current_state_topic, self._on_current_joint_state, 10
        )
        self._timer = self.create_timer(1.0 / send_rate_hz, self._send_pending_goal)

        self.get_logger().info(
            f"Target JointState bridge ready: {target_topic} -> {action_name}"
        )

    def _on_target_joint_state(self, msg: JointState) -> None:
        positions = self._extract_positions(msg)
        if positions is None:
            return
        if self._require_target_change and self._first_target_positions is None:
            self._first_target_positions = positions
            self.get_logger().info(
                "Initial target JointState captured; waiting for an operator change."
            )
            return
        if self._first_target_positions is not None:
            startup_delta = max(
                abs(current - initial)
                for current, initial in zip(positions, self._first_target_positions)
            )
            if startup_delta < self._min_delta:
                return
        command_positions = self._merge_unchanged_joints_with_current(positions)
        if self._last_sent_positions is not None:
            max_delta = max(
                abs(current - last)
                for current, last in zip(command_positions, self._last_sent_positions)
            )
            if max_delta < self._min_delta:
                return
        if self._pending_positions is not None:
            pending_delta = max(
                abs(current - pending)
                for current, pending in zip(command_positions, self._pending_positions)
            )
            if pending_delta < self._min_delta:
                return
        self._pending_positions = command_positions
        self._pending_updated_ns = self.get_clock().now().nanoseconds

    def _on_current_joint_state(self, msg: JointState) -> None:
        positions = self._extract_positions(msg, warn_on_missing=False)
        if positions is not None:
            self._current_positions = positions

    def _extract_positions(self, msg: JointState, *, warn_on_missing: bool = True) -> Optional[List[float]]:
        if msg.name:
            name_to_position = dict(zip(msg.name, msg.position))
            missing = [name for name in self._joint_names if name not in name_to_position]
            if missing:
                if warn_on_missing:
                    self.get_logger().warn(
                        f"Target JointState missing joints: {', '.join(missing)}",
                        throttle_duration_sec=2.0,
                    )
                return None
            return [float(name_to_position[name]) for name in self._joint_names]

        if len(msg.position) < len(self._joint_names):
            if warn_on_missing:
                self.get_logger().warn(
                    "Target JointState has no names and not enough positions.",
                    throttle_duration_sec=2.0,
                )
            return None
        return [float(value) for value in msg.position[: len(self._joint_names)]]

    def _merge_unchanged_joints_with_current(self, gui_positions: List[float]) -> List[float]:
        if (
            not self._hold_unchanged_joints
            or self._first_target_positions is None
            or self._current_positions is None
        ):
            return gui_positions

        merged = []
        for gui_value, initial_value, current_value in zip(
            gui_positions,
            self._first_target_positions,
            self._current_positions,
        ):
            if abs(gui_value - initial_value) < self._min_delta:
                merged.append(float(current_value))
            else:
                merged.append(float(gui_value))
        return merged

    def _send_pending_goal(self) -> None:
        if self._pending_positions is None or self._goal_in_flight:
            return
        now_ns = self.get_clock().now().nanoseconds
        if (
            self._pending_updated_ns is not None
            and now_ns - self._pending_updated_ns < self._target_settle_time_ns
        ):
            return
        if (
            self._last_result_ns is not None
            and now_ns - self._last_result_ns < self._result_cooldown_ns
        ):
            return
        if not self._client.server_is_ready():
            self.get_logger().warn(
                "FollowJointTrajectory action server is not ready.",
                throttle_duration_sec=2.0,
            )
            return

        positions = self._pending_positions
        self._pending_positions = None
        self._pending_updated_ns = None

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(seconds=self._duration_sec).to_msg()

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._joint_names
        goal.trajectory.points = [point]

        self._goal_in_flight = True
        self._active_positions = positions
        self.get_logger().info(
            "GUI target debug: "
            f"target_deg={self._format_deg(self._radians_to_degrees(positions))}, "
            f"duration_s={self._duration_sec:.2f}"
        )
        send_future = self._client.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._goal_in_flight = False
            self._active_positions = None
            self.get_logger().warn("FollowJointTrajectory goal was rejected.")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        result = future.result().result
        self._goal_in_flight = False
        self._last_result_ns = self.get_clock().now().nanoseconds
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            self._active_positions = None
            self.get_logger().warn(
                f"FollowJointTrajectory finished with error_code={result.error_code}."
            )
            return
        self._last_sent_positions = self._active_positions
        self._active_positions = None

    def _radians_to_degrees(self, values: List[float]) -> List[float]:
        return [float(value) * 180.0 / math.pi for value in values]

    def _format_deg(self, values: List[float]) -> str:
        return "[" + ", ".join(
            f"J{index + 1}={float(value):+.2f}"
            for index, value in enumerate(values[: len(self._joint_names)])
        ) + "]"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetJointStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
