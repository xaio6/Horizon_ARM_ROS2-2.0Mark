from __future__ import annotations

import importlib

import rclpy
from horizon_arm_interfaces.srv import VisualGrasp
from rclpy.node import Node

from .common import prepare_sdk_import


class VisualGraspServer(Node):
    """Expose VisualGraspSDK as a ROS2 service with a dry-run health path."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_visual_grasp_server")

        self.declare_parameter("sdk_root", "")
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("service_name", "/horizon_arm/visual_grasp")

        self._sdk = None
        self._service = self.create_service(
            VisualGrasp,
            str(self.get_parameter("service_name").value),
            self._on_visual_grasp,
        )
        self.get_logger().info(
            "Visual grasp service ready on "
            + str(self.get_parameter("service_name").value)
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

    def _get_sdk(self):
        if self._sdk is not None:
            return self._sdk
        sdk_root = str(self.get_parameter("sdk_root").value)
        prepare_sdk_import(sdk_root)
        embodied_sdk = importlib.import_module("Embodied_SDK")
        sdk_cls = getattr(embodied_sdk, "VisualGraspSDK")
        self._sdk = sdk_cls(camera_id=int(self.get_parameter("camera_id").value))
        return self._sdk


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualGraspServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
