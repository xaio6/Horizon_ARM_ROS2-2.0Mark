#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import List, Optional, Sequence

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


def _identity() -> List[List[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _matmul4(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> List[List[float]]:
    out = [[0.0] * 4 for _ in range(4)]
    for row in range(4):
        for col in range(4):
            out[row][col] = sum(float(left[row][k]) * float(right[k][col]) for k in range(4))
    return out


def _invert_se3(transform: Sequence[Sequence[float]]) -> List[List[float]]:
    rot_t = [[float(transform[col][row]) for col in range(3)] for row in range(3)]
    trans = [float(transform[row][3]) for row in range(3)]
    inv = _identity()
    for row in range(3):
        for col in range(3):
            inv[row][col] = rot_t[row][col]
        inv[row][3] = -sum(rot_t[row][k] * trans[k] for k in range(3))
    return inv


def _quat_to_rot(x: float, y: float, z: float, w: float) -> List[List[float]]:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def _rot_delta_deg(expected: Sequence[Sequence[float]], actual: Sequence[Sequence[float]]) -> float:
    rel = [[0.0] * 3 for _ in range(3)]
    for row in range(3):
        for col in range(3):
            rel[row][col] = sum(float(expected[k][row]) * float(actual[k][col]) for k in range(3))
    trace = float(rel[0][0] + rel[1][1] + rel[2][2])
    cos_angle = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    return math.degrees(math.acos(cos_angle))


def _dh_transform(theta_rad: float, d_m: float, a_m: float, alpha_rad: float) -> List[List[float]]:
    ct = math.cos(theta_rad)
    st = math.sin(theta_rad)
    ca = math.cos(alpha_rad)
    sa = math.sin(alpha_rad)
    return [
        [ct, -st * ca, st * sa, a_m * ct],
        [st, ct * ca, -ct * sa, a_m * st],
        [0.0, sa, ca, d_m],
        [0.0, 0.0, 0.0, 1.0],
    ]


class RvizPoseConsistencyMonitor(Node):
    def __init__(self) -> None:
        super().__init__("horizon_arm_rviz_pose_consistency_monitor")

        self.declare_parameter(
            "joint_names",
            ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        )
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tip_link", "tcp_link")
        self.declare_parameter("check_rate_hz", 1.0)
        self.declare_parameter("position_tolerance_mm", 8.0)
        self.declare_parameter("orientation_tolerance_deg", 5.0)
        self.declare_parameter("dh_config_path", "")
        self.declare_parameter("tcp_offset_mm", [0.0, 0.0, 0.0])

        self._joint_names = list(self.get_parameter("joint_names").value)
        self._joint_positions: Optional[List[float]] = None
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._tip_link = str(self.get_parameter("tip_link").value)
        self._position_tolerance_mm = float(self.get_parameter("position_tolerance_mm").value)
        self._orientation_tolerance_deg = float(self.get_parameter("orientation_tolerance_deg").value)
        self._check_period = 1.0 / max(0.2, float(self.get_parameter("check_rate_hz").value))
        self._tcp_offset_mm = [float(value) for value in self.get_parameter("tcp_offset_mm").value]

        self._d_mm, self._a_mm, self._alpha_deg, self._joint_offsets_deg = self._load_dh_config(
            str(self.get_parameter("dh_config_path").value)
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._joint_sub = self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            self._on_joint_state,
            10,
        )
        self._timer = self.create_timer(self._check_period, self._check_consistency)
        self._last_status: Optional[bool] = None
        self._alignment_transform: Optional[List[List[float]]] = None

        self.get_logger().info(
            "RViz pose consistency monitor ready: "
            f"{self._base_frame} -> {self._tip_link}, "
            f"position_tol_mm={self._position_tolerance_mm:.1f}, "
            f"orientation_tol_deg={self._orientation_tolerance_deg:.1f}"
        )

    def _load_dh_config(self, explicit_path: str) -> tuple[List[float], List[float], List[float], List[float]]:
        config_path = explicit_path.strip()
        if not config_path:
            config_dir = os.environ.get("HORIZONARM_CONFIG_DIR", "").strip()
            if config_dir:
                config_path = str(Path(config_dir) / "dh_parameters_config.json")
        if not config_path:
            raise RuntimeError("dh_config_path is empty and HORIZONARM_CONFIG_DIR is not set")

        with open(config_path, "r", encoding="utf-8") as handle:
            raw = json.load(handle)

        dh = raw.get("dh_parameters", raw)
        d_mm = [float(value) for value in dh["d"]]
        a_mm = [float(value) for value in dh["a"]]
        alpha_deg = [float(value) for value in dh["alpha_deg"]]
        joint_offsets_deg = [float(value) for value in raw.get("joint_offsets", [0.0] * 6)]
        if not all(len(values) == 6 for values in [d_mm, a_mm, alpha_deg, joint_offsets_deg]):
            raise RuntimeError("DH config must contain exactly 6 joints")
        return d_mm, a_mm, alpha_deg, joint_offsets_deg

    def _on_joint_state(self, msg: JointState) -> None:
        if not msg.name:
            if len(msg.position) >= len(self._joint_names):
                self._joint_positions = [float(value) for value in msg.position[: len(self._joint_names)]]
            return

        name_to_pos = dict(zip(msg.name, msg.position))
        missing = [name for name in self._joint_names if name not in name_to_pos]
        if missing:
            return
        self._joint_positions = [float(name_to_pos[name]) for name in self._joint_names]

    def _check_consistency(self) -> None:
        if self._joint_positions is None:
            return

        try:
            tf_msg = self._tf_buffer.lookup_transform(
                self._base_frame,
                self._tip_link,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warning(f"Pose consistency monitor waiting for TF: {exc}", throttle_duration_sec=5.0)
            return

        dh_pose = self._compute_dh_fk(self._joint_positions)
        tf_pos_mm = [
            float(tf_msg.transform.translation.x) * 1000.0,
            float(tf_msg.transform.translation.y) * 1000.0,
            float(tf_msg.transform.translation.z) * 1000.0,
        ]
        tf_rot = _quat_to_rot(
            float(tf_msg.transform.rotation.x),
            float(tf_msg.transform.rotation.y),
            float(tf_msg.transform.rotation.z),
            float(tf_msg.transform.rotation.w),
        )
        tf_pose = _identity()
        for row in range(3):
            for col in range(3):
                tf_pose[row][col] = tf_rot[row][col]
        tf_pose[0][3] = tf_pos_mm[0]
        tf_pose[1][3] = tf_pos_mm[1]
        tf_pose[2][3] = tf_pos_mm[2]

        if self._alignment_transform is None:
            self._alignment_transform = _matmul4(tf_pose, _invert_se3(dh_pose))
            self.get_logger().info(
                "RViz pose consistency baseline calibrated from current pose. "
                "Subsequent checks will detect motion semantic drift between DH and URDF/TF."
            )
            return

        aligned_dh_pose = _matmul4(self._alignment_transform, dh_pose)

        pos_err_mm = [
            aligned_dh_pose[0][3] - tf_pos_mm[0],
            aligned_dh_pose[1][3] - tf_pos_mm[1],
            aligned_dh_pose[2][3] - tf_pos_mm[2],
        ]
        pos_norm_mm = math.sqrt(sum(value * value for value in pos_err_mm))
        rot_err_deg = _rot_delta_deg(
            [
                [aligned_dh_pose[row][col] for col in range(3)]
                for row in range(3)
            ],
            tf_rot,
        )

        ok = pos_norm_mm <= self._position_tolerance_mm and rot_err_deg <= self._orientation_tolerance_deg
        if ok and self._last_status is True:
            return

        joint_deg = [math.degrees(value) for value in self._joint_positions]
        message = (
            "RViz pose consistency "
            + ("PASS: " if ok else "WARN: ")
            + f"joint_deg={self._format_list(joint_deg)} "
            + f"aligned_dh_tip_mm={self._format_list([aligned_dh_pose[0][3], aligned_dh_pose[1][3], aligned_dh_pose[2][3]])} "
            + f"tf_tip_mm={self._format_list(tf_pos_mm)} "
            + f"pos_err_mm={self._format_list(pos_err_mm)} "
            + f"pos_norm_mm={pos_norm_mm:.2f} "
            + f"rot_err_deg={rot_err_deg:.2f}"
        )
        if ok:
            self.get_logger().info(message)
        else:
            self.get_logger().warning(message)
        self._last_status = ok

    def _compute_dh_fk(self, joint_positions_rad: Sequence[float]) -> List[List[float]]:
        transform = _identity()
        for index in range(6):
            theta_rad = float(joint_positions_rad[index]) + math.radians(self._joint_offsets_deg[index])
            d_m = self._d_mm[index] / 1000.0
            a_m = self._a_mm[index] / 1000.0
            alpha_rad = math.radians(self._alpha_deg[index])
            transform = _matmul4(transform, _dh_transform(theta_rad, d_m, a_m, alpha_rad))
        tcp_transform = _identity()
        tcp_transform[0][3] = self._tcp_offset_mm[0] / 1000.0
        tcp_transform[1][3] = self._tcp_offset_mm[1] / 1000.0
        tcp_transform[2][3] = self._tcp_offset_mm[2] / 1000.0
        transform = _matmul4(transform, tcp_transform)
        return [
            [transform[row][col] * 1000.0 if col == 3 and row < 3 else transform[row][col] for col in range(4)]
            for row in range(4)
        ]

    def _format_list(self, values: Sequence[float]) -> str:
        return "[" + ", ".join(f"{float(value):+.2f}" for value in values) + "]"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RvizPoseConsistencyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
