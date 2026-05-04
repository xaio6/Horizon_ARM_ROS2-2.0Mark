from __future__ import annotations

import importlib
import json
from typing import Any

import rclpy
from horizon_arm_interfaces.srv import (
    DetectTarget,
    PickHSV,
    VisionConfig,
    VisualGrasp,
    VisualGraspEx,
)
from rclpy.node import Node

from .common import prepare_sdk_import, spin_node_until_shutdown
from .vision_fallback import (
    capture_frame,
    detect_frame_targets,
    parse_options_json,
    resolve_model_path,
    sample_hsv,
)


class VisualGraspServer(Node):
    """Expose VisualGraspSDK as a ROS2 service with a dry-run health path."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_visual_grasp_server")

        self.declare_parameter("sdk_root", "")
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("service_name", "/horizon_arm/visual_grasp")
        self.declare_parameter("ex_service_name", "/horizon_arm/visual_grasp_ex")
        self.declare_parameter("config_service_name", "/horizon_arm/vision_config")
        self.declare_parameter("pick_hsv_service_name", "/horizon_arm/pick_hsv")
        self.declare_parameter("detect_service_name", "/horizon_arm/detect_target")

        self._sdk = None
        self._vision_config = {}
        self._service = self.create_service(
            VisualGrasp,
            str(self.get_parameter("service_name").value),
            self._on_visual_grasp,
        )
        self._ex_service = self.create_service(
            VisualGraspEx,
            str(self.get_parameter("ex_service_name").value),
            self._on_visual_grasp_ex,
        )
        self._config_service = self.create_service(
            VisionConfig,
            str(self.get_parameter("config_service_name").value),
            self._on_vision_config,
        )
        self._pick_hsv_service = self.create_service(
            PickHSV,
            str(self.get_parameter("pick_hsv_service_name").value),
            self._on_pick_hsv,
        )
        self._detect_service = self.create_service(
            DetectTarget,
            str(self.get_parameter("detect_service_name").value),
            self._on_detect_target,
        )
        self.get_logger().info(
            "Visual grasp service ready on "
            + str(self.get_parameter("service_name").value)
        )
        self.get_logger().info(
            "Visual enhanced services ready on "
            + ", ".join(
                [
                    str(self.get_parameter("ex_service_name").value),
                    str(self.get_parameter("config_service_name").value),
                    str(self.get_parameter("pick_hsv_service_name").value),
                    str(self.get_parameter("detect_service_name").value),
                ]
            )
        )

    def _on_visual_grasp(self, request, response):
        try:
            sdk = self._get_sdk()
            if bool(request.dry_run):
                response.success = True
                response.message = "visual grasp wrapper health check passed"
                return response

            if bool(request.use_bbox):
                ok = bool(
                    sdk.grasp_at_bbox(
                        float(request.x1),
                        float(request.y1),
                        float(request.x2),
                        float(request.y2),
                    )
                )
                response.success = ok
                response.message = (
                    "visual grasp bbox command completed"
                    if ok
                    else "visual grasp bbox command failed"
                )
            else:
                ok = bool(sdk.grasp_at_pixel(float(request.u), float(request.v)))
                response.success = ok
                response.message = (
                    "visual grasp pixel command completed"
                    if ok
                    else "visual grasp pixel command failed"
                )
        except Exception as exc:
            response.success = False
            response.message = f"visual grasp failed: {exc}"
        return response

    def _on_visual_grasp_ex(self, request, response):
        try:
            sdk = self._get_sdk()
            self._apply_grasp_params_if_available(sdk, request)
            mode = str(request.mode or request.pipeline).strip().lower()
            if bool(request.dry_run):
                response.success = True
                response.message = f"visual grasp ex dry-run accepted: {mode}"
                response.target_xyz = []
                response.target_rpy = list(request.pre_grasp_rpy)
                response.result_json = json.dumps(
                    self._request_to_public_dict(request),
                    ensure_ascii=False,
                )
                return response

            detection_payload: dict[str, Any] = {}
            target_xyz = []

            if bool(request.use_bbox):
                ok = bool(
                    sdk.grasp_at_bbox(
                        float(request.x1),
                        float(request.y1),
                        float(request.x2),
                        float(request.y2),
                    )
                )
            elif bool(request.use_click) or mode in ("click", "pixel"):
                ok = bool(sdk.grasp_at_pixel(float(request.u), float(request.v)))
                target_xyz = self._estimate_target_xyz(sdk, float(request.u), float(request.v))
            else:
                detection_payload = self._detect_payload(
                    pipeline=str(request.pipeline or mode),
                    target_class=str(request.target_class),
                    conf_thres=float(self._vision_value("conf_thres", 0.5)),
                    iou_thres=float(self._vision_value("iou_thres", 0.45)),
                    use_hsv=bool(request.use_hsv),
                )
                detections = detection_payload.get("detections", [])
                if not detections:
                    response.success = False
                    response.message = "visual grasp ex failed: no target detected"
                    response.target_xyz = []
                    response.target_rpy = list(request.pre_grasp_rpy)
                    response.result_json = json.dumps(
                        {
                            "request": self._request_to_public_dict(request),
                            "detection": detection_payload,
                        },
                        ensure_ascii=False,
                    )
                    return response
                first = detections[0]
                bbox = [float(v) for v in first.get("bbox", [])[:4]]
                center = [float(v) for v in first.get("center", [])[:2]]
                if len(bbox) == 4:
                    ok = bool(sdk.grasp_at_bbox(*bbox))
                elif len(center) == 2:
                    ok = bool(sdk.grasp_at_pixel(center[0], center[1]))
                else:
                    raise ValueError("detected target has no usable bbox or center")
                if len(center) == 2:
                    target_xyz = self._estimate_target_xyz(sdk, center[0], center[1])

            response.success = ok
            response.message = (
                "visual grasp ex command completed"
                if ok
                else "visual grasp ex command failed"
            )
            response.target_xyz = target_xyz
            response.target_rpy = list(request.pre_grasp_rpy)
            response.result_json = json.dumps(
                {
                    "request": self._request_to_public_dict(request),
                    "detection": detection_payload,
                },
                ensure_ascii=False,
            )
        except Exception as exc:
            response.success = False
            response.message = f"visual grasp ex failed: {exc}"
            response.target_xyz = []
            response.target_rpy = []
            response.result_json = "{}"
        return response

    def _on_vision_config(self, request, response):
        command = str(request.command).strip().lower() or "get"
        try:
            if command in ("set", "update"):
                self._vision_config.update(self._request_to_public_dict(request))
                response.success = True
                response.message = "vision config updated"
            elif command in ("get", "status"):
                response.success = True
                response.message = "vision config queried"
            elif command == "clear":
                self._vision_config.clear()
                response.success = True
                response.message = "vision config cleared"
            else:
                response.success = False
                response.message = "unsupported command; use set/get/status/clear"
            response.config_json = json.dumps(self._vision_config, ensure_ascii=False)
        except Exception as exc:
            response.success = False
            response.message = f"vision config failed: {exc}"
            response.config_json = "{}"
        return response

    def _on_pick_hsv(self, request, response):
        try:
            sdk = self._get_sdk()
            method = getattr(sdk, "pick_hsv", None) or getattr(sdk, "sample_hsv", None)
            if callable(method):
                payload = method(
                    float(request.u),
                    float(request.v),
                    int(request.window_size),
                    bool(request.use_depth_filter),
                    float(request.depth_min_m),
                    float(request.depth_max_m),
                )
            else:
                frame = self._capture_frame(sdk)
                payload = sample_hsv(
                    frame,
                    float(request.u),
                    float(request.v),
                    int(request.window_size),
                )
            response.success = bool(payload.get("success", True))
            response.h = int(payload.get("h", 0))
            response.s = int(payload.get("s", 0))
            response.v = int(payload.get("v", 0))
            response.h_min = int(payload.get("h_min", 0))
            response.h_max = int(payload.get("h_max", 0))
            response.s_min = int(payload.get("s_min", 0))
            response.s_max = int(payload.get("s_max", 0))
            response.v_min = int(payload.get("v_min", 0))
            response.v_max = int(payload.get("v_max", 0))
            response.depth_m = float(payload.get("depth_m", 0.0))
            response.message = str(payload.get("message", "HSV sampled"))
        except Exception as exc:
            response.success = False
            response.h = response.s = response.v = 0
            response.h_min = response.h_max = 0
            response.s_min = response.s_max = 0
            response.v_min = response.v_max = 0
            response.depth_m = 0.0
            response.message = f"pick HSV failed: {exc}"
        return response

    def _on_detect_target(self, request, response):
        try:
            sdk = self._get_sdk()
            method = getattr(sdk, "detect_target", None)
            if callable(method):
                payload = method(
                    pipeline=str(request.pipeline),
                    target_class=str(request.target_class),
                    conf_thres=float(request.conf_thres),
                    use_hsv=bool(request.use_hsv),
                    use_depth=bool(request.use_depth),
                    depth_min_m=float(request.depth_min_m),
                    depth_max_m=float(request.depth_max_m),
                )
            else:
                payload = self._detect_payload(
                    pipeline=str(request.pipeline),
                    target_class=str(request.target_class),
                    conf_thres=float(request.conf_thres),
                    iou_thres=float(self._vision_value("iou_thres", 0.45)),
                    use_hsv=bool(request.use_hsv),
                )
            detections = payload.get("detections", [])
            response.success = bool(payload.get("success", True))
            response.count = int(payload.get("count", len(detections)))
            response.bboxes = [float(v) for det in detections for v in det.get("bbox", [])]
            response.centers = [
                float(v) for det in detections for v in det.get("center", [])
            ]
            response.scores = [float(det.get("score", 0.0)) for det in detections]
            response.class_names = [str(det.get("class_name", "")) for det in detections]
            response.depths_m = [float(det.get("depth_m", 0.0)) for det in detections]
            response.message = str(payload.get("message", "target detected"))
        except Exception as exc:
            response.success = False
            response.count = 0
            response.bboxes = []
            response.centers = []
            response.scores = []
            response.class_names = []
            response.depths_m = []
            response.message = f"detect target failed: {exc}"
        return response

    def _get_sdk(self):
        if self._sdk is not None:
            return self._sdk
        sdk_root = str(self.get_parameter("sdk_root").value)
        prepare_sdk_import(sdk_root)
        embodied_sdk = importlib.import_module("Embodied_SDK")
        sdk_cls = getattr(embodied_sdk, "VisualGraspSDK")
        self._sdk = sdk_cls(camera_id=int(self.get_parameter("camera_id").value))
        return self._sdk

    def _apply_grasp_params_if_available(self, sdk, request) -> None:
        method = getattr(sdk, "set_grasp_params", None)
        if not callable(method):
            return
        kwargs = {}
        if float(request.grasp_depth_m) > 0.0:
            kwargs["grasp_depth"] = float(request.grasp_depth_m) * 1000.0
        if len(request.pre_grasp_rpy) >= 3:
            kwargs["yaw"] = float(request.pre_grasp_rpy[0])
            kwargs["pitch"] = float(request.pre_grasp_rpy[1])
            kwargs["roll"] = float(request.pre_grasp_rpy[2])
        if kwargs:
            method(**kwargs)

    def _capture_frame(self, sdk):
        method = getattr(sdk, "_capture_single_frame", None)
        if callable(method):
            frame = method()
            if frame is not None:
                return frame
        return capture_frame(int(self.get_parameter("camera_id").value))

    def _detect_payload(
        self,
        *,
        pipeline: str,
        target_class: str,
        conf_thres: float,
        iou_thres: float,
        use_hsv: bool,
    ) -> dict[str, Any]:
        frame = self._capture_frame(self._get_sdk())
        normalized_pipeline = str(pipeline or "").strip().lower() or "yolo"
        if use_hsv and normalized_pipeline in ("yolo", "detect", "depth"):
            normalized_pipeline = "hybrid"
        hsv_range = self._current_hsv_range()
        model_path = ""
        try:
            model_path = resolve_model_path(
                explicit_path=str(self._vision_value("model_path", "")),
                sdk_root=str(self.get_parameter("sdk_root").value),
                config=self._vision_config,
            )
        except Exception:
            model_path = ""
        detections = detect_frame_targets(
            frame,
            pipeline=normalized_pipeline,
            target_class=target_class,
            conf_thres=float(conf_thres or self._vision_value("conf_thres", 0.5)),
            iou_thres=float(iou_thres or self._vision_value("iou_thres", 0.45)),
            hsv_range=hsv_range,
            model_path=model_path,
        )
        return {
            "success": True,
            "count": len(detections),
            "detections": detections,
            "message": "target detected" if detections else "no target detected",
        }

    def _current_hsv_range(self) -> list[int]:
        keys = [
            "hsv_h_min",
            "hsv_h_max",
            "hsv_s_min",
            "hsv_s_max",
            "hsv_v_min",
            "hsv_v_max",
        ]
        values = [int(self._vision_value(key, 0)) for key in keys]
        return values if any(values) else [0, 179, 0, 255, 0, 255]

    def _vision_value(self, key: str, default: Any) -> Any:
        value = self._vision_config.get(key, default)
        return default if value in (None, "") else value

    def _estimate_target_xyz(self, sdk, u: float, v: float) -> list[float]:
        try:
            module = importlib.import_module("Embodied_SDK.Horizon_Core.gateway")
            embodied_internal = module.get_embodied_internal_module()
            current_pose = embodied_internal._get_current_arm_pose()
            if current_pose is None:
                return []
            calib = embodied_internal._load_calibration_params()
            if not calib:
                return []
            grasp_params = embodied_internal._get_grasp_params()
            world = embodied_internal._convert_pixel_to_world_coords(
                float(u),
                float(v),
                calib,
                current_pose,
                tcp_x=float(grasp_params.get("tcp_offset_x", 0.0)),
                tcp_y=float(grasp_params.get("tcp_offset_y", 0.0)),
                tcp_z=float(grasp_params.get("tcp_offset_z", 0.0)),
            )
            if world is None:
                return []
            return [float(value) for value in list(world)[:3]]
        except Exception:
            del sdk
            return []

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
    node = VisualGraspServer()
    spin_node_until_shutdown(node, lambda: rclpy.spin(node))


if __name__ == "__main__":
    main()
