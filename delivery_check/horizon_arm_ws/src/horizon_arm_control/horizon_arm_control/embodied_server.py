from __future__ import annotations

import importlib
import json
import sys
import types

import rclpy
from horizon_arm_interfaces.srv import EmbodiedCommand, EmbodiedInstruction
from rclpy.node import Node

from .common import prepare_sdk_import, spin_node_until_shutdown


class EmbodiedServer(Node):
    """Expose EmbodiedSDK through a ROS2 service with safe health command."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_embodied_server")

        self.declare_parameter("sdk_root", "")
        self.declare_parameter("service_name", "/horizon_arm/embodied_instruction")
        self.declare_parameter(
            "command_service_name",
            "/horizon_arm/embodied_command",
        )
        self.declare_parameter("provider", "alibaba")
        self.declare_parameter("model", "qwen-turbo")
        self.declare_parameter("control_mode", "real_only")
        self.declare_parameter("config_path", "")

        self._sdk = None
        self._sdk_signature = None
        self._service = self.create_service(
            EmbodiedInstruction,
            str(self.get_parameter("service_name").value),
            self._on_embodied_instruction,
        )
        self._command_service = self.create_service(
            EmbodiedCommand,
            str(self.get_parameter("command_service_name").value),
            self._on_embodied_command,
        )
        self.get_logger().info(
            "Embodied instruction service ready on "
            + str(self.get_parameter("service_name").value)
        )
        self.get_logger().info(
            "Embodied command service ready on "
            + str(self.get_parameter("command_service_name").value)
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

    def _on_embodied_command(self, request, response):
        command = str(request.command).strip().lower() or "run"
        try:
            options = self._parse_options_json(str(request.options_json))
            sdk = self._get_sdk(
                provider=str(request.provider).strip() or None,
                model=str(request.model).strip() or None,
                control_mode=str(request.control_mode).strip() or None,
                config_path=str(options.get("config_path", "")).strip() or None,
            )

            if command in ("health", "functions"):
                payload = sdk.get_available_functions()
                response.success = True
                response.message = "embodied functions queried"
                response.result_json = json.dumps(payload, ensure_ascii=False)
                return response

            if command == "actions":
                payload = sdk.get_available_actions()
                response.success = True
                response.message = "embodied actions queried"
                response.result_json = json.dumps(payload, ensure_ascii=False)
                return response

            if command == "history":
                payload = sdk.get_history()
                response.success = True
                response.message = "embodied history queried"
                response.result_json = json.dumps(payload, ensure_ascii=False)
                return response

            if command == "clear_history":
                sdk.clear_history()
                response.success = True
                response.message = "embodied history cleared"
                response.result_json = "{}"
                return response

            if command in ("emergency_stop", "estop"):
                sdk.emergency_stop()
                response.success = True
                response.message = "embodied emergency stop triggered"
                response.result_json = "{}"
                return response

            if command == "set_emergency_stop":
                sdk.set_emergency_stop_flag(True)
                response.success = True
                response.message = "embodied emergency stop flag set"
                response.result_json = "{}"
                return response

            if command == "clear_emergency_stop":
                sdk.clear_emergency_stop_flag()
                response.success = True
                response.message = "embodied emergency stop flag cleared"
                response.result_json = "{}"
                return response

            if command in ("run", "stream"):
                instruction = str(request.instruction).strip()
                if not instruction:
                    raise ValueError("instruction is required for run/stream")

                if bool(request.stream) or command == "stream":
                    events = []

                    def _progress_handler(message: str):
                        events.append({"type": "progress", "message": str(message)})

                    def _completion_handler(result):
                        events.append({"type": "completion", "result": result})

                    sdk.run_nl_instruction_stream(
                        instruction,
                        progress_handler=_progress_handler,
                        completion_handler=_completion_handler,
                    )
                    response.success = True
                    response.message = "embodied stream instruction started"
                    response.result_json = json.dumps(events, ensure_ascii=False)
                    return response

                result = sdk.run_nl_instruction(instruction)
                response.success = bool(result.get("success", True))
                response.message = str(
                    result.get("message", "embodied instruction completed")
                )
                response.result_json = json.dumps(result, ensure_ascii=False)
                return response

            response.success = False
            response.message = (
                "unsupported command; use health/functions/actions/history/"
                "clear_history/run/stream/emergency_stop/set_emergency_stop/"
                "clear_emergency_stop"
            )
            response.result_json = "{}"
        except Exception as exc:
            response.success = False
            response.message = f"embodied command failed: {exc}"
            response.result_json = "{}"
        return response

    def _get_sdk(self, *, provider=None, model=None, control_mode=None, config_path=None):
        resolved_provider = provider or str(self.get_parameter("provider").value)
        resolved_model = model or str(self.get_parameter("model").value)
        resolved_control_mode = control_mode or str(
            self.get_parameter("control_mode").value
        )
        resolved_config_path = config_path
        if resolved_config_path is None:
            resolved_config_path = str(self.get_parameter("config_path").value) or None
        signature = (
            str(resolved_provider),
            str(resolved_model),
            str(resolved_control_mode),
            str(resolved_config_path or ""),
        )
        if self._sdk is not None and self._sdk_signature == signature:
            return self._sdk
        sdk_root = str(self.get_parameter("sdk_root").value)
        prepare_sdk_import(sdk_root)
        self._install_optional_dependency_stubs()
        embodied_sdk = importlib.import_module("Embodied_SDK")
        sdk_cls = getattr(embodied_sdk, "EmbodiedSDK")
        self._sdk = sdk_cls(
            provider=resolved_provider,
            model=resolved_model,
            control_mode=resolved_control_mode,
            config_path=resolved_config_path,
        )
        self._sdk_signature = signature
        return self._sdk

    def _install_optional_dependency_stubs(self) -> None:
        if "dotenv" not in sys.modules:
            dotenv_module = types.ModuleType("dotenv")

            def _load_dotenv(*args, **kwargs):
                return False

            dotenv_module.load_dotenv = _load_dotenv
            sys.modules["dotenv"] = dotenv_module

    def _parse_options_json(self, value: str) -> dict:
        text = str(value).strip()
        if not text:
            return {}
        payload = json.loads(text)
        if not isinstance(payload, dict):
            raise ValueError("options_json must be a JSON object")
        return payload


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EmbodiedServer()
    spin_node_until_shutdown(node, lambda: rclpy.spin(node))


if __name__ == "__main__":
    main()
