from __future__ import annotations

import importlib

import rclpy
from horizon_arm_interfaces.srv import FollowGraspControl
from rclpy.node import Node

from .common import prepare_sdk_import


class FollowGraspServer(Node):
    """Expose FollowGraspSDK control and health commands as a ROS2 service."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_follow_grasp_server")

        self.declare_parameter("sdk_root", "")
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("service_name", "/horizon_arm/follow_grasp_control")
        self.declare_parameter("default_target_class", "person")
        self.declare_parameter("default_conf_thres", 0.35)
        self.declare_parameter("default_interval_sec", 0.1)

        self._sdk = None
        self._service = self.create_service(
            FollowGraspControl,
            str(self.get_parameter("service_name").value),
            self._on_follow_grasp_control,
        )
        self.get_logger().info(
            "Follow grasp service ready on "
            + str(self.get_parameter("service_name").value)
        )

    def destroy_node(self) -> bool:
        try:
            if self._sdk is not None and bool(self._sdk.is_following()):
                self._sdk.stop_follow_grasp()
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
                sdk.configure_follow(
                    target_class=target_class,
                    conf_thres=conf_thres,
                    interval=interval_sec,
                )
                response.success = True
                response.running = bool(sdk.is_following())
                response.message = (
                    "follow grasp wrapper health check passed"
                    if command == "health"
                    else "follow grasp wrapper status queried"
                )
                return response

            if command == "start":
                sdk.configure_follow(
                    target_class=target_class,
                    conf_thres=conf_thres,
                    interval=interval_sec,
                )
                sdk.start_follow_grasp(
                    target_class=target_class,
                    conf_thres=conf_thres,
                    interval=interval_sec,
                )
                response.success = True
                response.running = bool(sdk.is_following())
                response.message = "follow grasp started"
                return response

            if command == "stop":
                sdk.stop_follow_grasp()
                response.success = True
                response.running = bool(sdk.is_following())
                response.message = "follow grasp stopped"
                return response

            response.success = False
            response.running = bool(sdk.is_following())
            response.message = "unsupported command; use health/status/start/stop"
        except Exception as exc:
            response.success = False
            response.running = False
            response.message = f"follow grasp control failed: {exc}"
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


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowGraspServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
