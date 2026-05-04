from __future__ import annotations

import importlib
import json

import rclpy
from horizon_arm_interfaces.srv import JoyconAdvancedControl, JoyconControl
from rclpy.node import Node

from .common import prepare_sdk_import, spin_node_until_shutdown


class JoyconServer(Node):
    """Expose JoyconSDK status/control hooks as a ROS2 service."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_joycon_server")

        self.declare_parameter("sdk_root", "")
        self.declare_parameter("service_name", "/horizon_arm/joycon_control")
        self.declare_parameter(
            "advanced_service_name",
            "/horizon_arm/joycon_advanced_control",
        )

        self._sdk = None
        self._service = self.create_service(
            JoyconControl,
            str(self.get_parameter("service_name").value),
            self._on_joycon_control,
        )
        self._advanced_service = self.create_service(
            JoyconAdvancedControl,
            str(self.get_parameter("advanced_service_name").value),
            self._on_joycon_advanced_control,
        )
        self.get_logger().info(
            "Joycon control service ready on "
            + str(self.get_parameter("service_name").value)
        )
        self.get_logger().info(
            "Joycon advanced control service ready on "
            + str(self.get_parameter("advanced_service_name").value)
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

    def _on_joycon_advanced_control(self, request, response):
        try:
            sdk = self._get_sdk()
            command = str(request.command).strip().lower() or "status"

            if command == "status":
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon advanced status queried",
                    sdk=sdk,
                )

            if command == "connect":
                left_ok, right_ok = sdk.connect_joycon()
                return self._fill_advanced_response(
                    response,
                    success=bool(left_ok or right_ok),
                    message=f"joycon connect attempted: left={left_ok}, right={right_ok}",
                    sdk=sdk,
                )

            if command == "disconnect":
                sdk.disconnect_joycon()
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon disconnected",
                    sdk=sdk,
                )

            if command == "start":
                sdk.start_control()
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon control started",
                    sdk=sdk,
                )

            if command == "stop":
                sdk.stop_control()
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon control stopped",
                    sdk=sdk,
                )

            if command == "pause":
                sdk.pause_control()
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon control paused",
                    sdk=sdk,
                )

            if command == "resume":
                sdk.resume_control()
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon control resumed",
                    sdk=sdk,
                )

            if command in ("emergency_stop", "estop"):
                sdk.emergency_stop()
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon emergency stop triggered",
                    sdk=sdk,
                )

            if command == "toggle_mode":
                sdk.toggle_mode()
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon basic mode toggled",
                    sdk=sdk,
                )

            if command == "set_mode":
                mode = str(request.mode).strip().lower()
                if mode in ("cartesian", "base", "tool"):
                    sdk.set_mode_cartesian()
                elif mode in ("joint", "joints"):
                    sdk.set_mode_joint()
                else:
                    raise ValueError("mode must be cartesian or joint")
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message=f"joycon mode set to {mode}",
                    sdk=sdk,
                )

            if command == "enable_attitude":
                mode = str(request.attitude_mode or request.mode or "joint").strip()
                sdk.enable_attitude(mode=mode)
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message=f"joycon attitude mode enabled: {mode}",
                    sdk=sdk,
                )

            if command == "disable_attitude":
                sdk.disable_attitude_mode()
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon attitude mode disabled",
                    sdk=sdk,
                )

            if command == "set_attitude_mode":
                mode = str(request.attitude_mode or request.mode).strip()
                if not mode:
                    raise ValueError("attitude_mode is required")
                sdk.set_attitude_mode(mode)
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message=f"joycon attitude mode set to {mode}",
                    sdk=sdk,
                )

            if command == "set_dual_attitude":
                sdk.set_dual_attitude_enabled(bool(request.enabled or request.dual_arm))
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon dual attitude mode updated",
                    sdk=sdk,
                )

            if command == "set_preferred_side":
                side = str(request.preferred_side).strip().lower()
                if side not in ("left", "right", "auto"):
                    raise ValueError("preferred_side must be left, right, or auto")
                sdk.set_preferred_side(side)
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message=f"joycon preferred side set to {side}",
                    sdk=sdk,
                )

            if command == "start_pose":
                sdk.move_to_joycon_start_pose(force=bool(request.enabled))
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon start pose requested",
                    sdk=sdk,
                )

            if command == "home":
                sdk.move_to_home()
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon home requested",
                    sdk=sdk,
                )

            if command == "hardware_zero":
                sdk.home_to_hardware_zero()
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon hardware zero requested",
                    sdk=sdk,
                )

            if command == "increase_speed":
                sdk.increase_speed()
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon speed increased",
                    sdk=sdk,
                )

            if command == "decrease_speed":
                sdk.decrease_speed()
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon speed decreased",
                    sdk=sdk,
                )

            if command == "configure_speed":
                if len(request.speed_levels) == 0:
                    raise ValueError("speed_levels is required")
                sdk.configure_speed_levels(
                    [float(value) for value in request.speed_levels],
                    int(request.speed_index),
                )
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon speed levels configured",
                    sdk=sdk,
                )

            if command == "configure_basic":
                if int(request.stick_deadzone) > 0:
                    sdk.set_stick_deadzone(int(request.stick_deadzone))
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon basic parameters configured",
                    sdk=sdk,
                )

            if command == "configure_cartesian":
                sdk.configure_cartesian(
                    position_step=float(request.cartesian_position_step),
                    rotation_step=float(request.cartesian_rotation_step),
                    max_speed=float(request.cartesian_max_speed),
                    max_angular_speed=float(request.cartesian_max_angular_speed),
                )
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon cartesian parameters configured",
                    sdk=sdk,
                )

            if command == "configure_joint":
                sdk.configure_joint(
                    angle_step=float(request.joint_angle_step),
                    max_speed=int(request.joint_max_speed),
                    acceleration=int(request.joint_acceleration),
                    deceleration=int(request.joint_deceleration),
                )
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon joint parameters configured",
                    sdk=sdk,
                )

            if command == "configure_workspace":
                sdk.configure_workspace(
                    min_radius=float(request.workspace_min_radius),
                    max_radius=float(request.workspace_max_radius),
                    min_z=float(request.workspace_min_z),
                    max_z=float(request.workspace_max_z),
                )
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon workspace parameters configured",
                    sdk=sdk,
                )

            if command == "input_status":
                return self._fill_advanced_response(
                    response,
                    success=True,
                    message="joycon input status queried",
                    sdk=sdk,
                )

            return self._fill_advanced_response(
                response,
                success=False,
                message=(
                    "unsupported command; use status/connect/disconnect/start/stop/"
                    "pause/resume/set_mode/enable_attitude/disable_attitude/"
                    "set_attitude_mode/set_dual_attitude/set_preferred_side/"
                    "start_pose/home/hardware_zero/emergency_stop/increase_speed/"
                    "decrease_speed/configure_speed/configure_basic/"
                    "configure_cartesian/configure_joint/configure_workspace/input_status"
                ),
                sdk=sdk,
            )
        except Exception as exc:
            response.success = False
            response.running = self._sdk_running()
            response.mode = ""
            response.attitude_mode = 0
            response.message = f"joycon advanced control failed: {exc}"
            response.status_json = "{}"
            response.input_json = "{}"
            return response

    def _sdk_running(self) -> bool:
        if self._sdk is None:
            return False
        running_attr = getattr(self._sdk, "running", False)
        return bool(running_attr() if callable(running_attr) else running_attr)

    def _fill_advanced_response(
        self,
        response,
        *,
        success: bool,
        message: str,
        sdk,
    ):
        status = self._safe_sdk_payload(sdk, "get_status")
        input_status = self._safe_sdk_payload(sdk, "get_input_status")
        response.success = bool(success)
        response.running = self._sdk_running()
        response.message = str(message)
        response.status_json = json.dumps(status, ensure_ascii=False)
        response.input_json = json.dumps(input_status, ensure_ascii=False)
        response.mode = str(status.get("control_mode", status.get("mode", "")))
        attitude = status.get("attitude_mode", 0)
        try:
            response.attitude_mode = int(attitude)
        except (TypeError, ValueError):
            response.attitude_mode = 0
        return response

    def _safe_sdk_payload(self, sdk, method_name: str) -> dict:
        method = getattr(sdk, method_name, None)
        if not callable(method):
            return {}
        try:
            payload = method()
            return payload if isinstance(payload, dict) else {"value": payload}
        except Exception as exc:
            return {"error": str(exc)}

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
    spin_node_until_shutdown(node, lambda: rclpy.spin(node))


if __name__ == "__main__":
    main()
