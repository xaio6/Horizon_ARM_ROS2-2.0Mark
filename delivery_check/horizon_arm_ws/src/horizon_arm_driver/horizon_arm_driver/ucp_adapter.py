from __future__ import annotations

import math
import importlib
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence


@dataclass(frozen=True)
class JointMapping:
    joint_names: List[str]
    motor_ids: List[int]
    reducer_ratios: List[float]
    directions: List[int]
    zero_offsets_deg: List[float]
    joint_limits_deg: List[List[float]]

    def validate(self) -> None:
        fields = [
            self.joint_names,
            self.motor_ids,
            self.reducer_ratios,
            self.directions,
            self.zero_offsets_deg,
            self.joint_limits_deg,
        ]
        if any(len(field) != 6 for field in fields):
            raise ValueError("Horizon Arm v2 mapping must describe exactly 6 joints")


class HorizonArmUcpAdapter:
    """Thin adapter around the Horizon Arm 2.0 UCP SDK."""

    def __init__(
        self,
        *,
        mapping: JointMapping,
        port: str,
        baudrate: int = 115200,
        sdk_root: str = "",
        hardware_enabled: bool = False,
        read_min_interval_sec: float = 0.5,
        read_error_backoff_sec: float = 2.5,
    ) -> None:
        mapping.validate()
        self.mapping = mapping
        self.port = port
        self.baudrate = int(baudrate)
        self.sdk_root = sdk_root
        self.hardware_enabled = bool(hardware_enabled)
        self.controllers = {}
        self._controller_cls = None
        self._last_joint_deg = [0.0] * 6
        self._connected = False
        self._motors_enabled = False
        self._warnings: List[str] = []
        self._last_read_had_error = False
        self._read_min_interval_sec = max(0.0, float(read_min_interval_sec))
        self._read_error_backoff_sec = max(0.0, float(read_error_backoff_sec))
        self._next_allowed_read_monotonic = 0.0
        self._gripper_controller = None
        self._gripper_sdk = None

    def connect(self) -> None:
        if not self.hardware_enabled:
            return

        self._prepare_sdk_import()
        from Embodied_SDK import close_all_shared_interfaces

        try:
            from Horizon_Core.Control_SDK.Control_Core import ZDTMotorController
        except ModuleNotFoundError:
            from Embodied_SDK.Horizon_Core.Control_SDK.Control_Core import ZDTMotorController

        self._controller_cls = ZDTMotorController
        self._patch_ucp_client_resync()
        self.controllers.clear()
        try:
            close_all_shared_interfaces()
        except Exception:
            pass
        self._flush_serial_buffers()

        try:
            for motor_id in self.mapping.motor_ids:
                ctrl = ZDTMotorController(
                    motor_id=int(motor_id),
                    port=self.port,
                    baudrate=self.baudrate,
                )
                # Follow the SDK contract: every controller connects itself and
                # lets the SDK connection pool manage the shared serial client.
                ctrl.connect()
                self.controllers[int(motor_id)] = ctrl
                time.sleep(0.05)
            self._connected = len(self.controllers) == len(self.mapping.motor_ids)
        except Exception:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        if self._gripper_controller is not None:
            try:
                self._gripper_controller.disconnect()
            except Exception:
                pass
        self._gripper_controller = None
        self._gripper_sdk = None
        for ctrl in list(self.controllers.values()):
            try:
                ctrl.disconnect()
            except Exception:
                pass
        try:
            from Embodied_SDK import close_all_shared_interfaces

            close_all_shared_interfaces()
        except Exception:
            pass
        self.controllers.clear()
        self._connected = False
        self._motors_enabled = False

    def enable(self) -> None:
        if not self.hardware_enabled or not self.controllers:
            return
        failures = []
        for motor_id in self.mapping.motor_ids:
            ctrl = self.controllers.get(int(motor_id))
            if ctrl is None:
                failures.append(f"motor {motor_id} controller missing")
                continue
            try:
                ctrl.enable(multi_sync=False)
                time.sleep(0.05)
            except Exception as exc:
                failures.append(f"motor {motor_id}: {exc}")
        if failures:
            raise RuntimeError("enable failed: " + "; ".join(failures))
        self._motors_enabled = True

    def disable(self) -> None:
        if not self.hardware_enabled or not self.controllers:
            return
        failures = []
        for motor_id in self.mapping.motor_ids:
            ctrl = self.controllers.get(int(motor_id))
            if ctrl is None:
                continue
            try:
                ctrl.disable(multi_sync=False)
                time.sleep(0.02)
            except Exception as exc:
                failures.append(f"motor {motor_id}: {exc}")
        if failures:
            self._remember_warning("disable partial failures: " + "; ".join(failures))
        self._motors_enabled = False

    def emergency_stop(self) -> None:
        for ctrl in list(self.controllers.values()):
            try:
                ctrl.emergency_stop()
            except Exception:
                pass
        self._motors_enabled = False

    def set_gripper_state(
        self,
        *,
        open: bool,
        current_ma: int = 1200,
        motor_id: int = 7,
    ) -> str:
        if not self.hardware_enabled:
            raise RuntimeError("hardware is disabled")
        gripper = self._get_gripper_sdk(motor_id=int(motor_id))
        current_ma = max(0, int(current_ma))
        if bool(open):
            gripper.open(current_ma=current_ma)
            return f"gripper opened with current_ma={current_ma}"
        gripper.close(current_ma=current_ma)
        return f"gripper closed with current_ma={current_ma}"

    def read_joint_positions_deg(self, *, force: bool = False) -> List[float]:
        if not self.hardware_enabled or not self.controllers:
            self._last_read_had_error = False
            return list(self._last_joint_deg)

        now = time.monotonic()
        if not force and self._next_allowed_read_monotonic > now:
            self._last_read_had_error = False
            return list(self._last_joint_deg)

        had_error = False
        values = []
        for index, motor_id in enumerate(self.mapping.motor_ids):
            ctrl = self.controllers.get(int(motor_id))
            if ctrl is None:
                had_error = True
                values.append(self._last_joint_deg[index])
                continue
            try:
                motor_deg = float(ctrl.read_parameters.get_position())
                values.append(self.motor_deg_to_joint_deg(index, motor_deg))
            except Exception as exc:
                had_error = True
                self._remember_warning(
                    f"read motor {motor_id} position failed: {exc}"
                )
                values.append(self._last_joint_deg[index])

        self._last_read_had_error = had_error
        if had_error:
            self._next_allowed_read_monotonic = now + self._read_error_backoff_sec
            return list(self._last_joint_deg)

        self._last_joint_deg = self._clip_joint_deg(values)
        self._next_allowed_read_monotonic = now + self._read_min_interval_sec
        return list(self._last_joint_deg)

    def send_joint_targets_deg(
        self, joint_deg: Sequence[float], speed_rpm: float = 200.0
    ) -> None:
        clipped = self._clip_joint_deg(joint_deg)

        if not self.hardware_enabled or not self.controllers:
            self._last_joint_deg = list(clipped)
            self._last_read_had_error = False
            return

        targets = {
            int(motor_id): self.joint_deg_to_motor_deg(index, clipped[index])
            for index, motor_id in enumerate(self.mapping.motor_ids)
        }
        self._controller_cls.y42_sync_position(
            self.controllers,
            targets=targets,
            speed=float(speed_rpm),
            is_absolute=True,
        )

    def upload_and_execute_trajectory(
        self, points: List[dict], timeout_ms: int = 5000
    ) -> None:
        if not points:
            return
        last_positions = points[-1].get("positions", [])
        if len(last_positions) >= 6 and (
            not self.hardware_enabled or not self.controllers
        ):
            self._last_joint_deg = [
                self.motor_deg_to_joint_deg(index, float(last_positions[index]))
                for index in range(6)
            ]
            self._last_read_had_error = False

        if not self.hardware_enabled or not self.controllers:
            return

        first_ctrl = self.controllers[int(self.mapping.motor_ids[0])]
        first_ctrl.upload_trajectory(points, timeout_ms=int(timeout_ms))
        first_ctrl.execute_trajectory(timeout_ms=2000)

    def joint_deg_to_motor_deg(self, index: int, joint_deg: float) -> float:
        return (
            float(self.mapping.directions[index])
            * float(joint_deg)
            * float(self.mapping.reducer_ratios[index])
            + float(self.mapping.zero_offsets_deg[index])
        )

    def motor_deg_to_joint_deg(self, index: int, motor_deg: float) -> float:
        direction = float(self.mapping.directions[index]) or 1.0
        ratio = float(self.mapping.reducer_ratios[index]) or 1.0
        return (
            float(motor_deg) - float(self.mapping.zero_offsets_deg[index])
        ) / (direction * ratio)

    @property
    def hardware_connected(self) -> bool:
        return bool(self.hardware_enabled and self._connected)

    @property
    def motors_enabled(self) -> bool:
        return bool(self.hardware_enabled and self._motors_enabled)

    @property
    def last_joint_positions_deg(self) -> List[float]:
        return list(self._last_joint_deg)

    @property
    def warnings(self) -> List[str]:
        return list(self._warnings)

    @property
    def last_read_had_error(self) -> bool:
        return bool(self._last_read_had_error)

    def _clip_joint_deg(self, joint_deg: Sequence[float]) -> List[float]:
        clipped = []
        for index, value in enumerate(list(joint_deg)[:6]):
            lo, hi = self.mapping.joint_limits_deg[index]
            clipped.append(max(float(lo), min(float(hi), float(value))))
        while len(clipped) < 6:
            clipped.append(0.0)
        return clipped

    def _prepare_sdk_import(self) -> None:
        if not self.sdk_root:
            return
        sdk_path = Path(self.sdk_root).expanduser().resolve()
        if sdk_path.is_dir():
            candidate_paths = [sdk_path, sdk_path / "Embodied_SDK"]
            for candidate in candidate_paths:
                if candidate.is_dir():
                    candidate_str = str(candidate)
                    if candidate_str not in sys.path:
                        sys.path.insert(0, candidate_str)

    def _patch_ucp_client_resync(self) -> None:
        module = None
        for module_name in (
            "Horizon_Core.Control_SDK.Control_Core.ucp_sdk.ucp_client",
            "Embodied_SDK.Horizon_Core.Control_SDK.Control_Core.ucp_sdk.ucp_client",
        ):
            try:
                module = importlib.import_module(module_name)
                break
            except ModuleNotFoundError:
                continue
        if module is None:
            return

        ucp_client_cls = getattr(module, "UcpClient", None)
        if ucp_client_cls is None or getattr(
            ucp_client_cls, "_horizon_ros2_resync_patch", False
        ):
            return

        def request_with_resync(
            client,
            motor_id: int,
            opcode: int,
            args: bytes = b"",
            timeout_ms: int = 1000,
            driver_type=None,
        ):
            if not client.ser:
                raise RuntimeError("serial not connected")

            with client._io_lock:
                try:
                    if hasattr(client.ser, "reset_input_buffer"):
                        client.ser.reset_input_buffer()
                    else:
                        waiting = int(getattr(client.ser, "in_waiting", 0) or 0)
                        if waiting > 0:
                            client.ser.read(waiting)
                except Exception:
                    pass

                driver = driver_type if driver_type is not None else client.driver_type
                payload = b"".join(
                    [
                        module.tlv(module.TlvTags.MOTOR_ID, struct.pack("<B", motor_id)),
                        module.tlv(module.TlvTags.DRIVER, struct.pack("<B", driver)),
                        module.tlv(module.TlvTags.OPCODE, struct.pack("<B", opcode)),
                        module.tlv(module.TlvTags.TIMEOUT_MS, struct.pack("<H", timeout_ms)),
                        module.tlv(module.TlvTags.ARGS, args),
                    ]
                )

                expected_seq = int(client.seq)
                frame = module.build_ucp_request(expected_seq, payload)
                client.ser.write(frame)
                client.ser.flush()

                next_seq = (expected_seq + 1) & 0xFFFF
                if next_seq == 0:
                    next_seq = 1

                deadline = time.time() + max(0.6, timeout_ms / 1000.0 + 0.5)
                last_issue = ""
                rpayload = b""

                while True:
                    remaining = deadline - time.time()
                    if remaining <= 0.0:
                        client.seq = next_seq
                        if last_issue:
                            raise TimeoutError(
                                f"waited for matching response after mismatch/serial issue: {last_issue} "
                                f"(expected type=0x{module.UCP_TYPE_RESPONSE:02X} seq={expected_seq})"
                            )
                        raise TimeoutError("waiting UCP response timed out")

                    try:
                        frame_type, rseq, rpayload = module.read_ucp_frame(
                            client.ser,
                            timeout_s=remaining,
                        )
                    except TimeoutError:
                        client.seq = next_seq
                        if last_issue:
                            raise TimeoutError(
                                f"waited for matching response after mismatch/serial issue: {last_issue} "
                                f"(expected type=0x{module.UCP_TYPE_RESPONSE:02X} seq={expected_seq})"
                            )
                        raise
                    except Exception as exc:
                        last_issue = str(exc)
                        time.sleep(0.01)
                        continue

                    if (
                        frame_type == module.UCP_TYPE_RESPONSE
                        and rseq == expected_seq
                    ):
                        break

                    last_issue = f"type=0x{frame_type:02X} seq={rseq}"

            tlvs = module.parse_tlvs(rpayload)
            status = tlvs.get(module.TlvTags.STATUS, b"\xFF")[0]
            err_bytes = tlvs.get(module.TlvTags.ERR_CODE, b"\x00\x00")
            err_code = err_bytes[0] | (err_bytes[1] << 8) if len(err_bytes) == 2 else 0
            data = tlvs.get(module.TlvTags.DATA, b"")
            diag = tlvs.get(module.TlvTags.DIAG, b"")

            client.seq = next_seq
            return module.UcpResponse(status=status, err_code=err_code, data=data, diag=diag)

        ucp_client_cls.request = request_with_resync
        ucp_client_cls._horizon_ros2_resync_patch = True

    def _flush_serial_buffers(self) -> None:
        try:
            import serial

            ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.05,
                write_timeout=0.05,
                exclusive=True,
            )
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            finally:
                ser.close()
        except Exception:
            pass

    def _remember_warning(self, message: str) -> None:
        if self._warnings and self._warnings[-1] == message:
            return
        self._warnings.append(message)
        self._warnings = self._warnings[-10:]

    def _get_gripper_sdk(self, *, motor_id: int):
        if self._gripper_sdk is not None:
            return self._gripper_sdk

        self._prepare_sdk_import()
        if self._controller_cls is None:
            raise RuntimeError("motor controller class is not ready")

        embodied_module = importlib.import_module("Embodied_SDK")
        gripper_cls = getattr(embodied_module, "ZDTGripperSDK")

        self._gripper_controller = self._controller_cls(
            motor_id=int(motor_id),
            port=self.port,
            baudrate=self.baudrate,
        )
        self._gripper_controller.connect()
        self._gripper_sdk = gripper_cls(motor=self._gripper_controller)
        return self._gripper_sdk


def radians_to_degrees(values: Iterable[float]) -> List[float]:
    return [float(v) * 180.0 / math.pi for v in values]


def degrees_to_radians(values: Iterable[float]) -> List[float]:
    return [float(v) * math.pi / 180.0 for v in values]
