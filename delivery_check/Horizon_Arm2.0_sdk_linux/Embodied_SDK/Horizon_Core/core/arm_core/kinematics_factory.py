#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kinematics factory for headless SDK and ROS2 use cases."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any, Dict, List

from Embodied_SDK.Horizon_Core.core.arm_core.kinematics import RobotKinematics


def _default_config() -> Dict[str, Any]:
    return {
        "dh_parameters": {
            "d": [130.5, 0.0, 0.0, 200.5, 0.0, 119.4],
            "a": [0.0, 0.0, 234.41, 47.94, 0.0, 0.0],
            "alpha_deg": [0.0, -90.0, 0.0, -90.0, 90.0, -90.0],
        },
        "joint_offsets": [0.0, 90.0, 0.0, 0.0, 0.0, 0.0],
        "joint_limits": {
            "1": [-120.0, 120.0],
            "2": [-60.0, 60.0],
            "3": [-60.0, 60.0],
            "4": [-160.0, 160.0],
            "5": [-90.0, 90.0],
            "6": [-160.0, 160.0],
        },
        "angle_unit": "deg",
        "enable_offset": True,
    }


def _candidate_config_paths() -> List[Path]:
    paths: List[Path] = []

    config_dir = os.environ.get("HORIZONARM_CONFIG_DIR", "").strip()
    if config_dir:
        paths.append(Path(config_dir) / "dh_parameters_config.json")

    data_root = os.environ.get("HORIZON_DATA_DIR", "").strip()
    if data_root:
        paths.append(Path(data_root) / "config" / "dh_parameters_config.json")

    sdk_root = Path(__file__).resolve().parents[5]
    paths.append(sdk_root / "config" / "dh_parameters_config.json")

    return paths


def load_kinematics_config(force_reload: bool = False) -> Dict[str, Any]:
    del force_reload
    config = _default_config()

    for path in _candidate_config_paths():
        try:
            if not path.exists():
                continue
            with open(path, "r", encoding="utf-8") as f:
                loaded = json.load(f) or {}
            if isinstance(loaded, dict):
                config.update(loaded)
            break
        except Exception:
            continue

    return config


def _normalized_joint_limits(config: Dict[str, Any]) -> List[List[float]]:
    raw = config.get("joint_limits", {})
    limits: List[List[float]] = []
    for idx in range(1, 7):
        value = raw.get(str(idx), [-180.0, 180.0]) if isinstance(raw, dict) else [-180.0, 180.0]
        if not isinstance(value, (list, tuple)) or len(value) != 2:
            value = [-180.0, 180.0]
        limits.append([float(value[0]), float(value[1])])
    return limits


def create_configured_kinematics() -> RobotKinematics:
    config = load_kinematics_config()
    dh = config.get("dh_parameters", {})

    d = [float(v) for v in dh.get("d", _default_config()["dh_parameters"]["d"])]
    a = [float(v) for v in dh.get("a", _default_config()["dh_parameters"]["a"])]
    alpha_deg = dh.get("alpha_deg", _default_config()["dh_parameters"]["alpha_deg"])
    alpha = [math.radians(float(v)) for v in alpha_deg]

    joint_offsets = [float(v) for v in config.get("joint_offsets", _default_config()["joint_offsets"])]
    joint_limits = [tuple(v) for v in _normalized_joint_limits(config)]
    angle_unit = str(config.get("angle_unit", "deg"))

    kinematics = RobotKinematics(
        d=d,
        a=a,
        alpha=alpha,
        joint_limits=joint_limits,
        angle_unit=angle_unit,
        joint_offsets=joint_offsets,
    )

    if bool(config.get("enable_offset", True)):
        try:
            kinematics.set_angle_offset(joint_offsets)
        except Exception:
            pass

    return kinematics
