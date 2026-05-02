from __future__ import annotations

import importlib

import rclpy
from horizon_arm_interfaces.srv import SetDigitalOutput
from rclpy.node import Node

from .common import prepare_sdk_import


class DigitalOutputServer(Node):
    """Expose the Windows-style IO SDK as a ROS2 digital output service."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_digital_output_server")

        self.declare_parameter("sdk_root", "")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("service_name", "/horizon_arm/set_digital_output")

        self._io_sdk = None
        self._service = self.create_service(
            SetDigitalOutput,
            str(self.get_parameter("service_name").value),
            self._on_set_digital_output,
        )
        self.get_logger().info(
            "Digital output service ready on "
            + str(self.get_parameter("service_name").value)
        )

    def destroy_node(self) -> bool:
        try:
            if self._io_sdk is not None:
                self._io_sdk.disconnect()
        except Exception:
            pass
        return super().destroy_node()

    def _on_set_digital_output(self, request, response):
        channel = int(request.channel)
        state = bool(request.state)
        if channel < 0 or channel > 7:
            response.success = False
            response.message = "channel must be between 0 and 7"
            return response

        try:
            io_sdk = self._get_io_sdk()
            ok = bool(io_sdk.set_do(channel, state))
            response.success = ok
            response.message = (
                f"DO{channel} set to {'HIGH' if state else 'LOW'}"
                if ok
                else f"failed to set DO{channel}"
            )
        except Exception as exc:
            response.success = False
            response.message = f"set digital output failed: {exc}"
        return response

    def _get_io_sdk(self):
        if self._io_sdk is not None:
            return self._io_sdk

        sdk_root = str(self.get_parameter("sdk_root").value)
        prepare_sdk_import(sdk_root)
        module = importlib.import_module("Embodied_SDK.io")
        io_sdk_cls = getattr(module, "IOSDK")
        self._io_sdk = io_sdk_cls(
            port=str(self.get_parameter("port").value),
            baudrate=int(self.get_parameter("baudrate").value),
            timeout=float(self.get_parameter("timeout_sec").value),
        )
        if not bool(self._io_sdk.connect()):
            raise RuntimeError(
                "failed to connect IO SDK on port "
                + str(self.get_parameter("port").value)
            )
        return self._io_sdk


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DigitalOutputServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
