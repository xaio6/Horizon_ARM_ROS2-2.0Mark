#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json

import rclpy
from rclpy.node import Node

from horizon_arm_control import HorizonArmRosSdk


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Basic Horizon Arm ROS2 SDK motion demo."
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Actually enable the arm and send a joint motion command.",
    )
    parser.add_argument(
        "--joints",
        default="0,-20,45,0,30,0",
        help="Comma-separated joint targets in degrees.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="Motion duration in seconds.",
    )
    args = parser.parse_args()

    joints = [float(item) for item in args.joints.split(",")]

    rclpy.init()
    node = Node("horizon_arm_basic_motion_demo")
    sdk = HorizonArmRosSdk(node)

    try:
        ready = sdk.wait_until_ready(
            include_instruction=True,
            include_digital_output=True,
            include_gripper=True,
        )
        print(json.dumps({"ready": ready}, ensure_ascii=False, indent=2))

        if not args.execute:
            print(
                "Dry mode complete. Re-run with --execute to enable the arm and move joints."
            )
            return

        enable_result = sdk.enable()
        print(json.dumps(enable_result.__dict__, ensure_ascii=False, indent=2))

        motion_result = sdk.move_joints_deg(joints, duration_sec=args.duration)
        print(json.dumps(motion_result.__dict__, ensure_ascii=False, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
