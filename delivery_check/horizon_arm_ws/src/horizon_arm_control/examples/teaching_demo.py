#!/usr/bin/env python3

from __future__ import annotations

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
    rclpy.init()
    node = Node("horizon_arm_teaching_demo")
    sdk = HorizonArmRosSdk(node)

    demo_program = {
        "name": "demo_validate_only",
        "points": [
            {
                "index": 1,
                "mode": "joint",
                "joint_angles": [0, -20, 45, 0, 30, 0],
                "interpolation_type": "joint",
                "interpolation_params": {
                    "max_speed": 90.0,
                    "acceleration": 180.0,
                },
            }
        ],
    }

    try:
        ready = sdk.wait_until_ready(include_extended_wrappers=True)
        print(json.dumps({"ready": ready}, ensure_ascii=False, indent=2))

        status = sdk.teach_jog("status")
        _pretty("Teach status", status.payload_json)

        jog = sdk.teach_jog_joint(2, 5.0, dry_run=True)
        _pretty("Teach joint jog dry-run", jog.payload_json)

        validate = sdk.run_teaching_program(
            command="validate",
            program_json=json.dumps(demo_program, ensure_ascii=False),
            dry_run=True,
        )
        _pretty("Teaching program validate", validate.result_json)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
