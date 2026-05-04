#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


def _default_sdk_root(workspace: Path) -> Path:
    candidate = workspace.parent / "Horizon_Arm2.0_sdk_linux"
    return candidate if candidate.exists() else Path("")


def _bool_text(value: bool) -> str:
    return "true" if value else "false"


def _default_preset_config_path(sdk_root: Path) -> Path:
    candidate = sdk_root / "config" / "embodied_config" / "preset_actions.json"
    return candidate if candidate.exists() else Path("")


def _default_preset_name(config_path: Path) -> str:
    if not config_path:
        return ""
    try:
        with config_path.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
    except Exception:
        return ""
    if not isinstance(payload, dict):
        return ""
    for name, item in payload.items():
        if isinstance(item, dict) and "joints" in item:
            return str(name)
    return ""


def _run(command: list[str], *, cwd: Path) -> int:
    print("+ " + " ".join(command), flush=True)
    completed = subprocess.run(command, cwd=str(cwd))
    return int(completed.returncode)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run the Horizon Arm ROS2 full acceptance check in one command."
    )
    parser.add_argument(
        "--real-hardware",
        action="store_true",
        help="Connect the arm/gripper/IO hardware and run live side-effect tests.",
    )
    parser.add_argument("--sdk-root", default="", help="Linux SDK root directory.")
    parser.add_argument("--report-dir", default="", help="Report output directory.")
    parser.add_argument("--arm-port", default="/dev/ttyACM0")
    parser.add_argument("--arm-baudrate", default="115200")
    parser.add_argument("--io-port", default="/dev/ttyUSB0")
    parser.add_argument("--io-baudrate", default="115200")
    parser.add_argument("--io-timeout-sec", default="1.0")
    parser.add_argument("--preset-config-path", default="")
    parser.add_argument("--preset-name", default="")
    parser.add_argument(
        "--step-delay-sec",
        default="2.0",
        help="Delay after each real hardware step, in seconds.",
    )
    parser.add_argument("--camera-id", default="0")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the ros2 launch command without executing it.",
    )
    parser.add_argument(
        "--no-camera",
        action="store_true",
        help="Mark camera-dependent live tests as not performed.",
    )
    parser.add_argument(
        "--no-io",
        action="store_true",
        help="Mark external IO live output as not performed.",
    )

    args, passthrough = parser.parse_known_args(argv)
    workspace = Path.cwd()
    sdk_root = Path(args.sdk_root).expanduser() if args.sdk_root else _default_sdk_root(workspace)
    report_dir = Path(args.report_dir).expanduser() if args.report_dir else workspace / "horizon_full_acceptance"
    preset_config_path = (
        Path(args.preset_config_path).expanduser()
        if args.preset_config_path
        else _default_preset_config_path(sdk_root)
    )
    preset_name = args.preset_name or _default_preset_name(preset_config_path)

    if not sdk_root:
        print(
            "ERROR: sdk_root was not provided and ../Horizon_Arm2.0_sdk_linux was not found.",
            file=sys.stderr,
        )
        return 2

    report_dir.mkdir(parents=True, exist_ok=True)

    launch_args = [
        "ros2",
        "launch",
        "horizon_arm_bringup",
        "acceptance_check.launch.py",
        f"sdk_root:={sdk_root}",
        f"report_dir:={report_dir}",
        f"real_hardware:={_bool_text(args.real_hardware)}",
        f"arm_port:={args.arm_port}",
        f"arm_baudrate:={args.arm_baudrate}",
        f"io_port:={args.io_port}",
        f"io_baudrate:={args.io_baudrate}",
        f"io_timeout_sec:={args.io_timeout_sec}",
        f"preset_config_path:={preset_config_path}",
        f"live_step_delay_sec:={args.step_delay_sec}",
        f"camera_id:={args.camera_id}",
        f"camera_hardware_available:={_bool_text(not args.no_camera)}",
        f"io_hardware_available:={_bool_text(not args.no_io)}",
        f"preset_name:={preset_name}",
    ]
    launch_args.extend(passthrough)

    print("Horizon Arm acceptance check")
    print(f"  mode: {'real hardware' if args.real_hardware else 'logic/no hardware'}")
    print(f"  sdk_root: {sdk_root}")
    print(f"  report_dir: {report_dir}")
    if preset_config_path:
        print(f"  preset_config_path: {preset_config_path}")
    if preset_name:
        print(f"  preset_name: {preset_name}")
    print(f"  step_delay_sec: {args.step_delay_sec}")
    if not args.real_hardware:
        print("  note: live hardware side-effect items will be reported as not performed.")
    if args.dry_run:
        print("+ " + " ".join(launch_args), flush=True)
        return 0

    return _run(launch_args, cwd=workspace)


if __name__ == "__main__":
    raise SystemExit(main())
