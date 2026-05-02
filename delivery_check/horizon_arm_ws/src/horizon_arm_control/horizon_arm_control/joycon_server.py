from __future__ import annotations

import importlib

import rclpy
from horizon_arm_interfaces.srv import JoyconControl
from rclpy.node import Node

from .common import prepare_sdk_import


class JoyconServer(Node):
    """Expose JoyconSDK status/control hooks as a ROS2 service."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_joycon_server")

        self.declare_parameter("sdk_root", "")
        self.declare_parameter("service_name", "/horizon_arm/joycon_control")

        self._sdk = None
        self._service = self.create_service(
            JoyconControl,
            str(self.get_parameter("service_name").value),
            self._on_joycon_control,
        )
        self.get_logger().info(
            "Joycon control service ready on "
            + str(self.get_parameter("service_name").value)
        )

    def destroy_node(self) -> bool:
        try:
            if self._sdk is not None and self._sdk_running():
                self._sdk.stop_control()
        except Exception:
            pass
        try:
            if self._sdk is not None:
                self._sdk.disconnect_joycon()
        except Exception:
            pass
        return super().destroy_node()

    def _on_joycon_control(self, request, response):
        try:
            sdk = self._get_sdk()
            command = str(request.command).strip().lower() or "status"

            if command == "status":
                response.success = True
                response.running = self._sdk_running()
                response.message = "joycon wrapper status queried"
                return response

            if command == "connect":
                left_ok, right_ok = sdk.connect_joycon()
                response.success = bool(left_ok or right_ok)
                response.running = self._sdk_running()
                response.message = (
                    f"joycon connect attempted: left={left_ok}, right={right_ok}"
                )
                return response

            if command == "disconnect":
                sdk.disconnect_joycon()
                response.success = True
                response.running = self._sdk_running()
                response.message = "joycon disconnected"
                return response

            if command == "start":
                sdk.start_control()
                response.success = True
                response.running = self._sdk_running()
                response.message = "joycon control started"
                return response

            if command == "stop":
                sdk.stop_control()
                response.success = True
                response.running = self._sdk_running()
                response.message = "joycon control stopped"
                return response

            response.success = False
            response.running = self._sdk_running()
            response.message = "unsupported command; use status/connect/disconnect/start/stop"
        except Exception as exc:
            response.success = False
            response.running = False
            response.message = f"joycon control failed: {exc}"
        return response

    def _sdk_running(self) -> bool:
        if self._sdk is None:
            return False
        running_attr = getattr(self._sdk, "running", False)
        return bool(running_attr() if callable(running_attr) else running_attr)

    def _get_sdk(self):
        if self._sdk is not None:
            return self._sdk
        sdk_root = str(self.get_parameter("sdk_root").value)
        prepare_sdk_import(sdk_root)
        embodied_sdk = importlib.import_module("Embodied_SDK")
        sdk_cls = getattr(embodied_sdk, "JoyconSDK")
        self._sdk = sdk_cls()
        return self._sdk


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JoyconServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
