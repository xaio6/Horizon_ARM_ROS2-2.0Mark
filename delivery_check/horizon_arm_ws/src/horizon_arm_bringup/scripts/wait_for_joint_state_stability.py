#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateStabilityWaiter(Node):
    def __init__(self) -> None:
        super().__init__("horizon_arm_joint_state_stability_waiter")

        self.declare_parameter("joint_state_topic", "/horizon_arm/joint_states")
        self.declare_parameter(
            "joint_names",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        )
        self.declare_parameter("stable_samples", 3)
        self.declare_parameter("max_delta_rad", 0.002)
        self.declare_parameter("timeout_sec", 8.0)
        self.declare_parameter("minimum_messages", 4)

        self._joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self._joint_names = list(self.get_parameter("joint_names").value)
        self._stable_samples = max(1, int(self.get_parameter("stable_samples").value))
        self._max_delta_rad = max(0.0, float(self.get_parameter("max_delta_rad").value))
        self._timeout_sec = max(1.0, float(self.get_parameter("timeout_sec").value))
        self._minimum_messages = max(2, int(self.get_parameter("minimum_messages").value))

        self._last_positions: Optional[List[float]] = None
        self._stable_count = 0
        self._message_count = 0
        self._done = False
        self._success = False
        self._started_at = time.time()

        self.create_subscription(
            JointState,
            self._joint_state_topic,
            self._on_joint_state,
            10,
        )

        self.get_logger().info(
            "Waiting for stable joint states: "
            f"topic={self._joint_state_topic}, stable_samples={self._stable_samples}, "
            f"max_delta_rad={self._max_delta_rad:.6f}, timeout_sec={self._timeout_sec:.1f}"
        )

    def _on_joint_state(self, msg: JointState) -> None:
        positions = self._extract_positions(msg)
        if positions is None:
            return

        self._message_count += 1
        if self._last_positions is None:
            self._last_positions = positions
            return

        max_delta = max(abs(positions[index] - self._last_positions[index]) for index in range(6))
        if max_delta <= self._max_delta_rad:
            self._stable_count += 1
        else:
            self._stable_count = 0

        self._last_positions = positions

        if self._message_count >= self._minimum_messages and self._stable_count >= self._stable_samples:
            self._done = True
            self._success = True
            self.get_logger().info(
                "Joint state startup window is stable. "
                f"messages={self._message_count}, stable_count={self._stable_count}, "
                f"joint_deg={self._format_deg(positions)}"
            )

    def spin_until_done(self) -> bool:
        while rclpy.ok() and not self._done:
            rclpy.spin_once(self, timeout_sec=0.2)
            if time.time() - self._started_at >= self._timeout_sec:
                self._done = True
                self._success = False
                last = self._format_deg(self._last_positions) if self._last_positions is not None else "[]"
                self.get_logger().warning(
                    "Timed out waiting for stable joint states; continuing startup anyway. "
                    f"messages={self._message_count}, stable_count={self._stable_count}, joint_deg={last}"
                )
        return self._success

    def _extract_positions(self, msg: JointState) -> Optional[List[float]]:
        if len(msg.position) < 6:
            return None

        if not msg.name:
            return [float(value) for value in msg.position[:6]]

        index_by_name = {name: index for index, name in enumerate(msg.name)}
        if any(name not in index_by_name for name in self._joint_names):
            return None

        return [
            float(msg.position[index_by_name[name]])
            for name in self._joint_names
        ]

    def _format_deg(self, positions_rad: Optional[List[float]]) -> str:
        if not positions_rad:
            return "[]"
        values = [math.degrees(value) for value in positions_rad]
        return "[" + ", ".join(f"J{index + 1}={value:+.2f}" for index, value in enumerate(values)) + "]"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateStabilityWaiter()
    try:
        node.spin_until_done()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
