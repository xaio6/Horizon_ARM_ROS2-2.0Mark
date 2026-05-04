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
        description="Vision config, detect, and dry-run grasp demo."
    )
    parser.add_argument("--pipeline", default="hsv")
    parser.add_argument("--target-class", default="red_block")
    parser.add_argument("--u", type=float, default=320.0)
    parser.add_argument("--v", type=float, default=240.0)
    parser.add_argument(
        "--hsv",
        default="0,12,80,255,60,255",
        help="HSV range as h_min,h_max,s_min,s_max,v_min,v_max",
    )
    args = parser.parse_args()

    hsv = [int(item) for item in args.hsv.split(",")]

    rclpy.init()
    node = Node("horizon_arm_vision_pipeline_demo")
    sdk = HorizonArmRosSdk(node)

    try:
        ready = sdk.wait_until_ready(include_extended_wrappers=True)
        print(json.dumps({"ready": ready}, ensure_ascii=False, indent=2))

        config_result = sdk.configure_vision(
            command="set",
            pipeline=args.pipeline,
            target_class=args.target_class,
            hsv=hsv,
            depth_min_m=0.15,
            depth_max_m=1.2,
        )
        _pretty("Vision config", config_result.payload_json)

        detect_result = sdk.detect_target(
            pipeline=args.pipeline,
            target_class=args.target_class,
            use_hsv=args.pipeline == "hsv",
            use_depth=True,
            depth_min_m=0.15,
            depth_max_m=1.2,
        )
        _pretty("Detect target", detect_result.payload_json)

        grasp_result = sdk.visual_grasp_ex(
            mode="click",
            pipeline="click",
            u=args.u,
            v=args.v,
            use_depth=True,
            dry_run=True,
            approach_height_m=0.08,
            grasp_depth_m=0.02,
        )
        _pretty("Visual grasp ex", grasp_result.payload_json)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
