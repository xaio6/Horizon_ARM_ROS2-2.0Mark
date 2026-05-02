#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""High-level Horizon Arm SDK facade.

This facade is convenient for desktop applications, but ROS2 and headless Linux
deployments typically only need a subset of the available modules. To make the
package usable on Ubuntu even when optional UI, vision, or gamepad
dependencies are missing, optional subsystems are imported lazily and
initialization failures are recorded instead of aborting the entire SDK.
"""

from __future__ import annotations

import importlib
from typing import Any, Dict, Optional


def _load_attr(module_name: str, attr_name: str) -> Any:
    module = importlib.import_module(module_name, package=__package__)
    return getattr(module, attr_name)


class HorizonArmSDK:
    """High-level SDK that wires together optional Horizon Arm capabilities."""

    def __init__(
        self,
        motors: Dict[int, Any],
        *,
        camera_id: int = 0,
        enable_vision: bool = True,
        enable_follow: bool = True,
        enable_embodied: bool = True,
        enable_joycon: bool = True,
        enable_io: bool = True,
        enable_digital_twin: bool = True,
    ) -> None:
        self.motors = motors
        self.camera_id = camera_id
        self.initialization_errors: Dict[str, Exception] = {}

        self.motion: Any = None
        self.vision: Optional[Any] = None
        self.follow: Optional[Any] = None
        self.embodied: Optional[Any] = None
        self.joycon: Optional[Any] = None
        self.io: Optional[Any] = None
        self.digital_twin: Optional[Any] = None

        print("\n[HorizonArmSDK] Initializing...")

        self.motion = self._create_component(
            "motion",
            ".motion",
            "MotionSDK",
            required=True,
            bind_motors=True,
        )

        if enable_vision:
            self.vision = self._create_component(
                "vision",
                ".visual_grasp",
                "VisualGraspSDK",
                init_kwargs={"camera_id": camera_id},
                bind_motors=True,
            )

        if enable_follow:
            self.follow = self._create_component(
                "follow",
                ".visual_grasp",
                "FollowGraspSDK",
                init_kwargs={"camera_id": camera_id},
                bind_motors=True,
            )

        if enable_embodied:
            self.embodied = self._create_component("embodied", ".embodied", "EmbodiedSDK")

        if enable_joycon:
            self.joycon = self._create_component("joycon", ".joycon", "JoyconSDK")
            if self.joycon is not None and hasattr(self.joycon, "bind_arm"):
                try:
                    self.joycon.bind_arm(motors)
                except Exception as exc:
                    self.initialization_errors["joycon"] = exc
                    self.joycon = None

        if enable_io:
            self.io = self._create_component("io", ".io", "IOSDK")

        if enable_digital_twin:
            self.digital_twin = self._create_component(
                "digital_twin",
                ".digital_twin",
                "DigitalTwinSDK",
            )

        if self.initialization_errors:
            failed = ", ".join(sorted(self.initialization_errors))
            print(f"[HorizonArmSDK] Optional components unavailable: {failed}")
        print("[HorizonArmSDK] Ready.")

    def _create_component(
        self,
        name: str,
        module_name: str,
        attr_name: str,
        *,
        required: bool = False,
        init_kwargs: Optional[Dict[str, Any]] = None,
        bind_motors: bool = False,
    ) -> Optional[Any]:
        try:
            cls = _load_attr(module_name, attr_name)
            component = cls(**(init_kwargs or {}))
            if bind_motors and hasattr(component, "bind_motors"):
                component.bind_motors(self.motors)
            return component
        except Exception as exc:
            self.initialization_errors[name] = exc
            if required:
                raise RuntimeError(f"Failed to initialize required component '{name}'") from exc
            return None

    def update_motors(self, motors: Dict[int, Any]) -> None:
        self.motors = motors

        if self.vision is not None:
            self.vision.bind_motors(motors)
        if self.follow is not None:
            self.follow.bind_motors(motors)
        if self.motion is not None:
            self.motion.bind_motors(motors)
        if self.joycon is not None and hasattr(self.joycon, "bind_arm"):
            try:
                self.joycon.bind_arm(motors)
            except Exception:
                pass

    def set_camera_id(self, camera_id: int) -> None:
        self.camera_id = camera_id

        if self.vision is not None:
            self.vision.camera_id = camera_id
        if self.follow is not None:
            self.follow.camera_id = camera_id

        try:
            from Embodied_SDK.Horizon_Core import gateway as horizon_gateway

            embodied_internal = horizon_gateway.get_embodied_internal_module()
            embodied_internal._set_camera_id(camera_id)
        except Exception:
            pass

    def get_kinematics_config(self, *, force_reload: bool = False) -> Dict[str, Any]:
        load_kinematics_config = _load_attr(
            "Embodied_SDK.Horizon_Core.core.arm_core.kinematics_factory",
            "load_kinematics_config",
        )
        return load_kinematics_config(force_reload=force_reload)
