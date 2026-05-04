#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json

import rclpy
from rclpy.node import Node

from horizon_arm_control import HorizonArmRosSdk


def _pretty(label: str, payload: str) -> None:
    try:
        obj = json.loads(payload)
        print(label)
        print(json.dumps(obj, ensure_ascii=False, indent=2))
    except json.JSONDecodeError:
        print(label)
        print(payload)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Embodied command demo for Horizon Arm ROS2."
    )
    parser.add_argument("--provider", default="")
    parser.add_argument("--model", default="")
    parser.add_argument("--control-mode", default="")
    parser.add_argument("--instruction", default="")
    parser.add_argument("--stream", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = Node("horizon_arm_embodied_demo")
    sdk = HorizonArmRosSdk(node)

    try:
        ready = sdk.wait_until_ready(include_extended_wrappers=True)
        print(json.dumps({"ready": ready}, ensure_ascii=False, indent=2))

        functions_result = sdk.embodied_command(
            "functions",
            provider=args.provider,
            model=args.model,
            control_mode=args.control_mode,
        )
        _pretty("Embodied functions", functions_result.payload_json)

        actions_result = sdk.embodied_command(
            "actions",
            provider=args.provider,
            model=args.model,
            control_mode=args.control_mode,
        )
        _pretty("Embodied actions", actions_result.payload_json)

        if args.instruction:
            run_result = sdk.embodied_command(
                "stream" if args.stream else "run",
                instruction=args.instruction,
                stream=args.stream,
                provider=args.provider,
                model=args.model,
                control_mode=args.control_mode,
            )
            _pretty("Embodied run", run_result.payload_json)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
