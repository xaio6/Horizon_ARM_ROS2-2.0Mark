from __future__ import annotations

import importlib
import json

import rclpy
from horizon_arm_interfaces.srv import EmbodiedInstruction
from rclpy.node import Node

from .common import prepare_sdk_import


class EmbodiedServer(Node):
    """Expose EmbodiedSDK through a ROS2 service with safe health command."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_embodied_server")

        self.declare_parameter("sdk_root", "")
        self.declare_parameter("service_name", "/horizon_arm/embodied_instruction")
        self.declare_parameter("provider", "alibaba")
        self.declare_parameter("model", "qwen-turbo")
        self.declare_parameter("control_mode", "real_only")
        self.declare_parameter("config_path", "")

        self._sdk = None
        self._service = self.create_service(
            EmbodiedInstruction,
            str(self.get_parameter("service_name").value),
            self._on_embodied_instruction,
        )
        self.get_logger().info(
            "Embodied instruction service ready on "
            + str(self.get_parameter("service_name").value)
        )

    def _on_embodied_instruction(self, request, response):
        command = str(request.command).strip()
        if not command:
            response.success = False
            response.message = "command is empty"
            response.result_json = "{}"
            return response

        try:
            sdk = self._get_sdk()
            lower = command.lower()
            if lower == "health":
                payload = sdk.get_available_functions()
                response.success = True
                response.message = "embodied wrapper health check passed"
                response.result_json = json.dumps(payload, ensure_ascii=False)
                return response

            if bool(request.stream):
                events = []

                def _progress_handler(message: str):
                    events.append({"type": "progress", "message": str(message)})

                def _completion_handler(result):
                    events.append({"type": "completion", "result": result})

                sdk.run_nl_instruction_stream(
                    command,
                    progress_handler=_progress_handler,
                    completion_handler=_completion_handler,
                )
                response.success = True
                response.message = "embodied stream instruction started"
                response.result_json = json.dumps(events, ensure_ascii=False)
                return response

            result = sdk.run_nl_instruction(command)
            response.success = bool(result.get("success", True))
            response.message = str(result.get("message", "embodied instruction completed"))
            response.result_json = json.dumps(result, ensure_ascii=False)
        except Exception as exc:
            response.success = False
            response.message = f"embodied instruction failed: {exc}"
            response.result_json = "{}"
        return response

    def _get_sdk(self):
        if self._sdk is not None:
            return self._sdk
        sdk_root = str(self.get_parameter("sdk_root").value)
        prepare_sdk_import(sdk_root)
        embodied_sdk = importlib.import_module("Embodied_SDK")
        sdk_cls = getattr(embodied_sdk, "EmbodiedSDK")
        self._sdk = sdk_cls(
            provider=str(self.get_parameter("provider").value),
            model=str(self.get_parameter("model").value),
            control_mode=str(self.get_parameter("control_mode").value),
            config_path=str(self.get_parameter("config_path").value) or None,
        )
        return self._sdk


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EmbodiedServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
