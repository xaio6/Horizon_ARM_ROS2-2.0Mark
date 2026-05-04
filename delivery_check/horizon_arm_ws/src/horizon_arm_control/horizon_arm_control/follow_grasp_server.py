from __future__ import annotations

import importlib
import json
import threading
import time
from typing import Any

import rclpy
from horizon_arm_interfaces.srv import FollowGraspControl, FollowTarget
from rclpy.node import Node

from .common import prepare_sdk_import, spin_node_until_shutdown
from .vision_fallback import (
    capture_frame,
    detect_frame_targets,
    parse_options_json,
    resolve_model_path,
)


class FollowGraspServer(Node):
    """Expose FollowGraspSDK control and health commands as a ROS2 service."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_follow_grasp_server")

        self.declare_parameter("sdk_root", "")
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("service_name", "/horizon_arm/follow_grasp_control")
        self.declare_parameter("target_service_name", "/horizon_arm/follow_target")
        self.declare_parameter("default_target_class", "person")
        self.declare_parameter("default_conf_thres", 0.35)
        self.declare_parameter("default_interval_sec", 0.1)

        self._sdk = None
        self._state_lock = threading.Lock()
        self._follow_thread = None
        self._follow_running = False
        self._stable_hits = 0
        self._last_target_center: list[float] = []
        self._follow_target_state = {
            "mode": "yolo",
            "pipeline": "yolo",
            "target_class": str(self.get_parameter("default_target_class").value),
            "conf_thres": float(self.get_parameter("default_conf_thres").value),
            "interval_sec": float(self.get_parameter("default_interval_sec").value),
            "follow_distance_m": 0.35,
            "deadband_px": 20.0,
            "max_linear_speed": 0.0,
            "max_angular_speed": 0.0,
            "use_depth": True,
            "auto_grasp": False,
            "options": {},
            "manual_target_ready": False,
        }
        self._service = self.create_service(
            FollowGraspControl,
            str(self.get_parameter("service_name").value),
            self._on_follow_grasp_control,
        )
        self._target_service = self.create_service(
            FollowTarget,
            str(self.get_parameter("target_service_name").value),
            self._on_follow_target,
        )
        self.get_logger().info(
            "Follow grasp service ready on "
            + str(self.get_parameter("service_name").value)
        )
        self.get_logger().info(
            "Follow target service ready on "
            + str(self.get_parameter("target_service_name").value)
        )

    def destroy_node(self) -> bool:
        try:
            self._stop_follow_loop()
        except Exception:
            pass
        return super().destroy_node()

    def _on_follow_grasp_control(self, request, response):
        try:
            sdk = self._get_sdk()
            command = str(request.command).strip().lower() or "status"
            target_class = (
                str(request.target_class).strip()
                or str(self.get_parameter("default_target_class").value)
            )
            conf_thres = (
                float(request.conf_thres)
                if float(request.conf_thres) > 0.0
                else float(self.get_parameter("default_conf_thres").value)
            )
            interval_sec = (
                float(request.interval_sec)
                if float(request.interval_sec) > 0.0
                else float(self.get_parameter("default_interval_sec").value)
            )

            if command in ("health", "status"):
                self._configure_follow_compat(
                    sdk,
                    target_class=target_class,
                    conf_thres=conf_thres,
                    interval_sec=interval_sec,
                )
                response.success = True
                response.running = self._is_follow_running(sdk)
                response.message = (
                    "follow grasp wrapper health check passed"
                    if command == "health"
                    else "follow grasp wrapper status queried"
                )
                return response

            if command == "start":
                self._update_follow_state(
                    mode="yolo",
                    pipeline="yolo",
                    target_class=target_class,
                    conf_thres=conf_thres,
                    interval_sec=interval_sec,
                )
                self._start_follow_loop(sdk)
                response.success = True
                response.running = self._is_follow_running(sdk)
                response.message = "follow grasp started"
                return response

            if command == "stop":
                self._stop_follow_loop(sdk)
                response.success = True
                response.running = self._is_follow_running(sdk)
                response.message = "follow grasp stopped"
                return response

            response.success = False
            response.running = self._is_follow_running(sdk)
            response.message = "unsupported command; use health/status/start/stop"
        except Exception as exc:
            response.success = False
            response.running = False
            response.message = f"follow grasp control failed: {exc}"
        return response

    def _on_follow_target(self, request, response):
        try:
            sdk = self._get_sdk()
            command = str(request.command).strip().lower() or "status"
            mode = str(request.mode).strip().lower() or "yolo"
            target_class = (
                str(request.target_class).strip()
                or str(self.get_parameter("default_target_class").value)
            )
            conf_thres = (
                float(request.conf_thres)
                if float(request.conf_thres) > 0.0
                else float(self.get_parameter("default_conf_thres").value)
            )
            interval_sec = (
                float(request.interval_sec)
                if float(request.interval_sec) > 0.0
                else float(self.get_parameter("default_interval_sec").value)
            )
            options = parse_options_json(str(request.options_json))

            if command in ("health", "status"):
                response.success = True
                response.running = self._is_follow_running(sdk)
                response.message = (
                    "follow target health check passed"
                    if command == "health"
                    else "follow target status queried"
                )
                response.state_json = self._follow_state_json(request, sdk)
                return response

            if command in ("start", "resume"):
                self._configure_follow_target_state(
                    sdk,
                    mode=mode,
                    target_class=target_class,
                    conf_thres=conf_thres,
                    interval_sec=interval_sec,
                    request=request,
                    options=options,
                )
                if mode == "manual" and not self._manual_target_ready():
                    self._initialize_manual_target_from_options(sdk, options)
                self._start_follow_loop(sdk)
                response.success = True
                response.running = self._is_follow_running(sdk)
                response.message = f"follow target started: {mode}"
                response.state_json = self._follow_state_json(request, sdk)
                return response

            if command in ("stop", "pause"):
                self._stop_follow_loop(sdk)
                response.success = True
                response.running = self._is_follow_running(sdk)
                response.message = f"follow target {command} completed"
                response.state_json = self._follow_state_json(request, sdk)
                return response

            if command == "set_target":
                self._initialize_manual_target_from_options(sdk, options)
                response.success = True
                response.running = self._is_follow_running(sdk)
                response.message = "manual target initialized"
                response.state_json = self._follow_state_json(request, sdk)
                return response

            response.success = False
            response.running = self._is_follow_running(sdk)
            response.message = "unsupported command; use health/status/start/stop/pause/resume/set_target"
            response.state_json = self._follow_state_json(request, sdk)
        except Exception as exc:
            response.success = False
            response.running = False
            response.message = f"follow target failed: {exc}"
            response.state_json = "{}"
        return response

    def _get_sdk(self):
        if self._sdk is not None:
            return self._sdk
        sdk_root = str(self.get_parameter("sdk_root").value)
        prepare_sdk_import(sdk_root)
        embodied_sdk = importlib.import_module("Embodied_SDK")
        sdk_cls = getattr(embodied_sdk, "FollowGraspSDK")
        self._sdk = sdk_cls(camera_id=int(self.get_parameter("camera_id").value))
        return self._sdk

    def _configure_follow_target_state(
        self,
        sdk,
        *,
        mode: str,
        target_class: str,
        conf_thres: float,
        interval_sec: float,
        request,
        options: dict[str, Any],
    ) -> None:
        self._configure_follow_compat(
            sdk,
            target_class=target_class,
            conf_thres=conf_thres,
            interval_sec=interval_sec,
        )
        self._apply_follow_options_to_sdk(sdk, options)
        self._update_follow_state(
            mode=mode,
            pipeline=str(request.pipeline or mode),
            target_class=target_class,
            conf_thres=conf_thres,
            interval_sec=interval_sec,
            follow_distance_m=float(request.follow_distance_m),
            deadband_px=float(request.deadband_px),
            max_linear_speed=float(request.max_linear_speed),
            max_angular_speed=float(request.max_angular_speed),
            use_depth=bool(request.use_depth),
            auto_grasp=bool(request.auto_grasp),
            options=options,
        )

    def _configure_follow_compat(
        self,
        sdk,
        *,
        target_class: str,
        conf_thres: float,
        interval_sec: float,
    ) -> None:
        method = getattr(sdk, "configure_follow", None)
        if not callable(method):
            return
        try:
            method(
                target_class=target_class,
                conf_thres=conf_thres,
                interval=interval_sec,
            )
        except TypeError:
            method(
                target_class=target_class,
                confidence_threshold=conf_thres,
                control_frequency=max(1, int(1.0 / max(0.001, interval_sec))),
            )

    def _apply_follow_options_to_sdk(self, sdk, options: dict[str, Any]) -> None:
        if hasattr(sdk, "set_follow_compensation"):
            sdk.set_follow_compensation(
                scale_x=float(options.get("scale_x", 1.0)),
                scale_y=float(options.get("scale_y", 1.0)),
                offset_x=float(options.get("offset_x", 0.0)),
                offset_y=float(options.get("offset_y", 0.0)),
            )
        if hasattr(sdk, "configure_follow"):
            try:
                sdk.configure_follow(
                    plane_mode=bool(options.get("plane_mode", True)),
                    min_bbox=int(options.get("min_bbox", 24)),
                )
            except TypeError:
                pass

    def _update_follow_state(self, **kwargs) -> None:
        with self._state_lock:
            self._follow_target_state.update(kwargs)

    def _manual_target_ready(self) -> bool:
        with self._state_lock:
            return bool(self._follow_target_state.get("manual_target_ready", False))

    def _set_manual_target_ready(self, value: bool) -> None:
        with self._state_lock:
            self._follow_target_state["manual_target_ready"] = bool(value)

    def _capture_frame(self, sdk):
        method = getattr(sdk, "_capture_single_frame", None)
        if callable(method):
            frame = method()
            if frame is not None:
                return frame
        return capture_frame(int(self.get_parameter("camera_id").value))

    def _initialize_manual_target_from_options(self, sdk, options: dict[str, Any]) -> None:
        method = getattr(sdk, "init_manual_target", None)
        if not callable(method):
            raise RuntimeError("FollowGraspSDK does not support manual target initialization")
        x1 = options.get("x1")
        y1 = options.get("y1")
        x2 = options.get("x2")
        y2 = options.get("y2")
        if None in (x1, y1, x2, y2):
            raise ValueError("options_json must contain x1, y1, x2, y2 for set_target")
        frame = self._capture_frame(sdk)
        ok = bool(method(frame, float(x1), float(y1), float(x2), float(y2)))
        if not ok:
            raise RuntimeError("manual target initialization failed")
        self._set_manual_target_ready(True)

    def _start_follow_loop(self, sdk) -> None:
        with self._state_lock:
            if self._follow_running:
                return
            self._follow_running = True
            self._stable_hits = 0
            self._last_target_center = []

        def _loop():
            try:
                while True:
                    with self._state_lock:
                        if not self._follow_running:
                            break
                        state = dict(self._follow_target_state)
                    try:
                        self._follow_iteration(sdk, state)
                    except Exception as exc:
                        self.get_logger().warning(
                            f"follow iteration failed: {exc}",
                            throttle_duration_sec=2.0,
                        )
                    time.sleep(max(0.02, float(state.get("interval_sec", 0.1))))
            finally:
                with self._state_lock:
                    self._follow_running = False

        self._follow_thread = threading.Thread(target=_loop, daemon=True)
        self._follow_thread.start()

    def _stop_follow_loop(self, sdk=None) -> None:
        del sdk
        with self._state_lock:
            self._follow_running = False
        current_thread = threading.current_thread()
        if (
            self._follow_thread
            and self._follow_thread.is_alive()
            and self._follow_thread is not current_thread
        ):
            self._follow_thread.join(timeout=2.0)
        self._follow_thread = None
        try:
            if self._sdk is not None and hasattr(self._sdk, "stop_follow_grasp"):
                self._sdk.stop_follow_grasp()
        except Exception:
            pass

    def _is_follow_running(self, sdk) -> bool:
        del sdk
        with self._state_lock:
            return bool(self._follow_running)

    def _follow_iteration(self, sdk, state: dict[str, Any]) -> None:
        frame = self._capture_frame(sdk)
        mode = str(state.get("mode", "yolo")).strip().lower() or "yolo"
        options = dict(state.get("options", {}))
        center = None

        if mode == "manual":
            center = self._manual_center_from_frame(sdk, frame)
        elif mode in ("hsv", "hybrid", "yolo", "depth"):
            center = self._detected_center_from_frame(sdk, frame, state)

        if center is None:
            with self._state_lock:
                self._stable_hits = 0
            return

        self._apply_follow_servo(sdk, center[0], center[1])
        with self._state_lock:
            self._last_target_center = [float(center[0]), float(center[1])]
            self._stable_hits += 1
            stable_hits = self._stable_hits

        if bool(state.get("auto_grasp", False)) and stable_hits >= int(options.get("stable_frames", 8)):
            grasp_method = getattr(sdk, "grasp_at_pixel", None)
            if callable(grasp_method):
                grasp_method(float(center[0]), float(center[1]))
            self._stop_follow_loop()

    def _manual_center_from_frame(self, sdk, frame):
        tracker = getattr(sdk, "_manual_tracker", None)
        if tracker is None or not hasattr(tracker, "update"):
            return None
        ok, center = tracker.update(frame)
        return center if ok else None

    def _detected_center_from_frame(self, sdk, frame, state: dict[str, Any]):
        mode = str(state.get("mode", "yolo")).strip().lower()
        options = dict(state.get("options", {}))
        pipeline = str(state.get("pipeline", mode)).strip().lower() or mode
        hsv_range = self._hsv_range_from_options(options)
        if mode == "hsv":
            if len(hsv_range) != 6:
                return None
            pipeline = "hsv"
        elif mode == "hybrid":
            pipeline = "hybrid"
        elif mode == "depth":
            pipeline = "yolo"
        model_path = ""
        try:
            model_path = resolve_model_path(
                explicit_path=str(options.get("model_path", "")),
                sdk_root=str(self.get_parameter("sdk_root").value),
            )
        except Exception:
            model_path = ""
        detections = detect_frame_targets(
            frame,
            pipeline=pipeline,
            target_class=str(state.get("target_class", "")),
            conf_thres=float(state.get("conf_thres", 0.35)),
            iou_thres=float(options.get("iou_thres", 0.45)),
            hsv_range=hsv_range,
            model_path=model_path,
        )
        if not detections:
            return None
        center = detections[0].get("center", [])
        return center if len(center) >= 2 else None

    def _apply_follow_servo(self, sdk, x: float, y: float) -> None:
        servo_method = getattr(sdk, "_apply_follow_servo", None)
        if callable(servo_method):
            servo_method(float(x), float(y))
            return
        follow_method = getattr(sdk, "follow_step", None)
        if callable(follow_method):
            frame = self._capture_frame(sdk)
            follow_method(frame)

    def _hsv_range_from_options(self, options: dict[str, Any]) -> list[int]:
        keys = [
            "hsv_h_min",
            "hsv_h_max",
            "hsv_s_min",
            "hsv_s_max",
            "hsv_v_min",
            "hsv_v_max",
        ]
        values = [int(options.get(key, 0)) for key in keys]
        return values if any(values) else []

    def _follow_state_json(self, request, sdk) -> str:
        payload = {
            "running": self._is_follow_running(sdk),
            "request": self._request_to_public_dict(request),
            "state": self._follow_target_state,
            "last_target_center": list(self._last_target_center),
        }
        return json.dumps(payload, ensure_ascii=False)

    def _request_to_public_dict(self, request) -> dict:
        payload = {}
        for field in getattr(request, "__slots__", []):
            name = field[1:] if field.startswith("_") else field
            value = self._to_jsonable(getattr(request, field))
            if isinstance(value, str) and name == "options_json":
                try:
                    value = parse_options_json(value)
                except Exception:
                    pass
            payload[name] = value
        return payload

    def _to_jsonable(self, value):
        if isinstance(value, (str, int, float, bool)) or value is None:
            return value
        if isinstance(value, (list, tuple)):
            return [self._to_jsonable(item) for item in value]
        if hasattr(value, "tolist") and callable(value.tolist):
            try:
                return self._to_jsonable(value.tolist())
            except Exception:
                pass
        if hasattr(value, "typecode") and hasattr(value, "__iter__"):
            try:
                return [self._to_jsonable(item) for item in list(value)]
            except Exception:
                pass
        return str(value)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowGraspServer()
    spin_node_until_shutdown(node, lambda: rclpy.spin(node))


if __name__ == "__main__":
    main()
