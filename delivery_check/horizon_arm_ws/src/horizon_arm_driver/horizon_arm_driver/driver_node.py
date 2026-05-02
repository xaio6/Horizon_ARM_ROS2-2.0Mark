from __future__ import annotations

import math
import time
from typing import List, Optional

import rclpy
from control_msgs.action import FollowJointTrajectory
from horizon_arm_interfaces.msg import ArmStatus
from horizon_arm_interfaces.srv import SetGripperState
from moveit_msgs.msg import DisplayTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from .ucp_adapter import (
    HorizonArmUcpAdapter,
    JointMapping,
    degrees_to_radians,
    radians_to_degrees,
)


class HorizonArmDriverNode(Node):
    """ROS2 hardware owner for Horizon Arm 2.0."""

    def __init__(self) -> None:
        super().__init__("horizon_arm_driver")

        self.declare_parameter("hardware_enabled", False)
        self.declare_parameter("sdk_root", "")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("state_publish_rate_hz", 5.0)
        self.declare_parameter("hardware_read_rate_hz", 1.0)
        self.declare_parameter("hardware_read_error_backoff_sec", 2.5)
        self.declare_parameter("status_publish_rate_hz", 1.0)
        self.declare_parameter("startup_settle_sec", 0.35)
        self.declare_parameter("trajectory_timeout_ms", 5000)
        self.declare_parameter("trajectory_debug_logging", True)
        self.declare_parameter("execution_goal_tolerance_deg", 3.0)
        self.declare_parameter("execution_noop_threshold_deg", 0.2)
        self.declare_parameter("execution_state_check_period_sec", 0.2)
        self.declare_parameter("execution_settle_timeout_sec", 2.0)
        self.declare_parameter("emergency_stop_on_execute_error", False)
        self.declare_parameter("gripper_motor_id", 7)
        self.declare_parameter("gripper_default_current_ma", 1200)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("canonical_joint_state_topic", "/horizon_arm/joint_states")
        self.declare_parameter("joint_names", [f"joint_{i}" for i in range(1, 7)])
        self.declare_parameter("motor_ids", [1, 2, 3, 4, 5, 6])
        self.declare_parameter("reducer_ratios", [50.0, 50.0, 50.0, 30.0, 30.0, 30.0])
        self.declare_parameter("directions", [-1, 1, 1, -1, 1, -1])
        self.declare_parameter("zero_offsets_deg", [0.0] * 6)
        self.declare_parameter("joint_limits_min_deg", [-120.0, -60.0, -60.0, -160.0, -90.0, -160.0])
        self.declare_parameter("joint_limits_max_deg", [120.0, 60.0, 60.0, 160.0, 90.0, 160.0])

        self.mapping = JointMapping(
            joint_names=list(self.get_parameter("joint_names").value),
            motor_ids=[int(v) for v in self.get_parameter("motor_ids").value],
            reducer_ratios=[float(v) for v in self.get_parameter("reducer_ratios").value],
            directions=[int(v) for v in self.get_parameter("directions").value],
            zero_offsets_deg=[float(v) for v in self.get_parameter("zero_offsets_deg").value],
            joint_limits_deg=self._load_joint_limits(),
        )

        self.adapter = HorizonArmUcpAdapter(
            mapping=self.mapping,
            port=str(self.get_parameter("port").value),
            baudrate=int(self.get_parameter("baudrate").value),
            sdk_root=str(self.get_parameter("sdk_root").value),
            hardware_enabled=bool(self.get_parameter("hardware_enabled").value),
            read_min_interval_sec=1.0
            / max(0.1, float(self.get_parameter("hardware_read_rate_hz").value)),
            read_error_backoff_sec=float(
                self.get_parameter("hardware_read_error_backoff_sec").value
            ),
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            10,
        )
        self.canonical_joint_state_pub = self.create_publisher(
            JointState,
            str(self.get_parameter("canonical_joint_state_topic").value),
            10,
        )
        self.status_pub = self.create_publisher(ArmStatus, "/horizon_arm/status", 10)
        self.display_plan_sub = self.create_subscription(
            DisplayTrajectory,
            "/display_planned_path",
            self._on_display_planned_path,
            10,
        )
        self._last_plan_final_deg: Optional[List[float]] = None
        self._last_plan_point_count = 0
        self._last_plan_stamp_sec = 0.0
        self.state_timer = None
        self.status_timer = None
        self.enable_srv = self.create_service(Trigger, "/horizon_arm/enable", self._on_enable)
        self.disable_srv = self.create_service(Trigger, "/horizon_arm/disable", self._on_disable)
        self.estop_srv = self.create_service(Trigger, "/horizon_arm/emergency_stop", self._on_emergency_stop)
        self.gripper_srv = self.create_service(
            SetGripperState,
            "/horizon_arm/set_gripper_state",
            self._on_set_gripper_state,
        )
        self.follow_action = ActionServer(
            self,
            FollowJointTrajectory,
            "/horizon_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute_follow_joint_trajectory,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )

        try:
            self.adapter.connect()
            if bool(self.get_parameter("hardware_enabled").value):
                self.adapter.enable()
                self.get_logger().info("Horizon Arm 2.0 UCP hardware connected and enabled.")
            else:
                self.get_logger().info("Hardware disabled; driver is running in simulation echo mode.")
            self._log_joint_mapping()
        except Exception as exc:
            self.get_logger().error(f"Failed to initialize Horizon Arm hardware: {exc}")
            raise

        startup_settle_sec = max(0.0, float(self.get_parameter("startup_settle_sec").value))
        if startup_settle_sec > 0.0:
            time.sleep(startup_settle_sec)

        publish_rate = float(self.get_parameter("state_publish_rate_hz").value)
        self.state_timer = self.create_timer(max(0.005, 1.0 / publish_rate), self._publish_joint_state)
        status_rate = float(self.get_parameter("status_publish_rate_hz").value)
        self.status_timer = self.create_timer(max(0.05, 1.0 / status_rate), self._publish_status)

    def _load_joint_limits(self) -> List[List[float]]:
        mins = [float(v) for v in self.get_parameter("joint_limits_min_deg").value]
        maxs = [float(v) for v in self.get_parameter("joint_limits_max_deg").value]
        if len(mins) != 6 or len(maxs) != 6:
            raise ValueError("joint_limits_min_deg and joint_limits_max_deg must both contain 6 values")
        return [[mins[index], maxs[index]] for index in range(6)]

    def destroy_node(self) -> bool:
        try:
            self.adapter.disconnect()
        finally:
            return super().destroy_node()

    def _goal_callback(self, goal_request) -> GoalResponse:
        names = list(goal_request.trajectory.joint_names)
        if names and set(names) != set(self.mapping.joint_names):
            self.get_logger().warning(f"Rejecting trajectory with unexpected joints: {names}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        del goal_handle
        self.get_logger().warning("Trajectory cancel requested.")
        return CancelResponse.ACCEPT

    def _execute_follow_joint_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        feedback = FollowJointTrajectory.Feedback()
        result = FollowJointTrajectory.Result()

        if not trajectory.points:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Trajectory contains no points."
            goal_handle.abort()
            return result

        try:
            start_deg = self.adapter.read_joint_positions_deg(force=True)
            requested_final_deg = self._point_to_joint_deg(
                trajectory.joint_names,
                trajectory.points[-1],
                fallback_deg=start_deg,
                clip=False,
            )
            final_deg = self.adapter._clip_joint_deg(requested_final_deg)
            move_delta_deg = self._subtract_deg(final_deg, start_deg)
            max_move_delta_deg = max(abs(value) for value in move_delta_deg)
            self._log_execute_request(
                trajectory.joint_names,
                trajectory.points,
                start_deg,
                requested_final_deg,
                final_deg,
            )

            noop_threshold_deg = max(
                0.0, float(self.get_parameter("execution_noop_threshold_deg").value)
            )
            if max_move_delta_deg <= noop_threshold_deg:
                if self._debug_logging_enabled():
                    self.get_logger().info(
                        "Execute request skipped as no-op: "
                        f"max_move_delta_deg={max_move_delta_deg:.3f}, "
                        f"threshold_deg={noop_threshold_deg:.3f}, "
                        f"target_deg={self._format_deg(final_deg)}"
                    )
                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = ""
                return result

            if len(trajectory.points) == 1:
                speed_rpm = self._estimate_single_point_speed_rpm(
                    final_deg, trajectory.points[0], start_deg
                )
                self._log_single_point_command(final_deg, speed_rpm)
                self.adapter.send_joint_targets_deg(final_deg, speed_rpm=speed_rpm)
            else:
                sdk_points = self._trajectory_to_sdk_points(
                    trajectory.joint_names, trajectory.points, start_deg
                )
                timeout_ms = int(self.get_parameter("trajectory_timeout_ms").value)
                self._log_trajectory_command(sdk_points)
                self.adapter.upload_and_execute_trajectory(
                    sdk_points, timeout_ms=timeout_ms
                )
                final_deg = [
                    self.adapter.motor_deg_to_joint_deg(
                        index, sdk_points[-1]["positions"][index]
                    )
                    for index in range(6)
                ]
            actual_deg, error_deg, reached = self._wait_for_goal_reached(
                final_deg,
                trajectory.points[-1],
            )
            feedback.joint_names = self.mapping.joint_names
            feedback.desired.positions = degrees_to_radians(final_deg)
            feedback.actual.positions = degrees_to_radians(actual_deg)
            goal_handle.publish_feedback(feedback)

            if not reached:
                self._stop_after_execute_error(
                    "final joint target was not reached within tolerance"
                )
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = (
                    "Final target not reached. "
                    f"target={self._format_deg(final_deg)} "
                    f"actual={self._format_deg(actual_deg)} "
                    f"error={self._format_deg(error_deg)}"
                )
                return result

            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = ""
            return result
        except Exception as exc:
            self.get_logger().error(f"Trajectory execution failed: {exc}")
            self._stop_after_execute_error(str(exc))
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(exc)
            goal_handle.abort()
            return result

    def _publish_joint_state(self) -> None:
        joint_deg = self.adapter.read_joint_positions_deg(force=False)
        if self.adapter.last_read_had_error:
            self.get_logger().warning(
                "Joint state read had timeout/error; publishing last known actual state for failed joints.",
                throttle_duration_sec=10.0,
            )
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.mapping.joint_names
        msg.position = degrees_to_radians(joint_deg)
        self.joint_state_pub.publish(msg)
        self.canonical_joint_state_pub.publish(msg)

    def _publish_status(self) -> None:
        msg = ArmStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.hardware_connected = self.adapter.hardware_connected
        msg.motors_enabled = self.adapter.motors_enabled
        msg.joint_position_deg = list(self.adapter.last_joint_positions_deg)
        msg.joint_velocity_deg_s = []
        msg.warnings = self.adapter.warnings
        self.status_pub.publish(msg)

    def _on_enable(self, request, response):
        del request
        try:
            self.adapter.enable()
            response.success = True
            response.message = "机械臂电机已使能。"
        except Exception as exc:
            response.success = False
            response.message = f"使能失败: {exc}"
        return response

    def _on_disable(self, request, response):
        del request
        try:
            self.adapter.disable()
            response.success = True
            response.message = "机械臂电机已失能。"
        except Exception as exc:
            response.success = False
            response.message = f"失能失败: {exc}"
        return response

    def _on_emergency_stop(self, request, response):
        del request
        try:
            self.adapter.emergency_stop()
            response.success = True
            response.message = "急停命令已下发。"
        except Exception as exc:
            response.success = False
            response.message = f"急停失败: {exc}"
        return response

    def _on_set_gripper_state(self, request, response):
        try:
            current_ma = int(request.current_ma)
            if current_ma <= 0:
                current_ma = int(self.get_parameter("gripper_default_current_ma").value)
            response.message = self.adapter.set_gripper_state(
                open=bool(request.open),
                current_ma=current_ma,
                motor_id=int(self.get_parameter("gripper_motor_id").value),
            )
            response.success = True
        except Exception as exc:
            response.success = False
            response.message = f"set gripper state failed: {exc}"
        return response

    def _trajectory_to_sdk_points(
        self,
        joint_names: List[str],
        points: List[JointTrajectoryPoint],
        start_joint_deg: List[float],
    ) -> List[dict]:
        last_time_ms = 0
        sdk_points = []
        previous_joint_deg = list(start_joint_deg)
        previous_motor_deg = [
            self.adapter.joint_deg_to_motor_deg(index, previous_joint_deg[index])
            for index in range(6)
        ]

        for point in points:
            target_deg = self._point_to_joint_deg(
                joint_names,
                point,
                fallback_deg=previous_joint_deg,
                clip=True,
            )
            target_motor_deg = [
                self.adapter.joint_deg_to_motor_deg(index, target_deg[index])
                for index in range(6)
            ]

            point_time_ms = int(point.time_from_start.sec * 1000 + point.time_from_start.nanosec / 1_000_000)
            interval_ms = max(1, point_time_ms - last_time_ms)
            last_time_ms = point_time_ms

            speeds = self._estimate_motor_speeds_rpm(previous_motor_deg, target_motor_deg, interval_ms)
            sdk_points.append(
                {
                    "interval_ms": interval_ms,
                    "positions": target_motor_deg,
                    "speeds": speeds,
                }
            )
            previous_joint_deg = target_deg
            previous_motor_deg = target_motor_deg

        return sdk_points

    def _point_to_joint_deg(
        self,
        joint_names: List[str],
        point: JointTrajectoryPoint,
        *,
        fallback_deg: Optional[List[float]] = None,
        clip: bool = True,
    ) -> List[float]:
        index_map = self._joint_index_map(joint_names)
        base_deg = (
            list(fallback_deg)
            if fallback_deg is not None
            else self.adapter.read_joint_positions_deg(force=False)
        )
        target_rad = degrees_to_radians(base_deg)
        for local_index, src_index in enumerate(index_map):
            if src_index >= 0 and src_index < len(point.positions):
                target_rad[local_index] = float(point.positions[src_index])
        target_deg = radians_to_degrees(target_rad)
        if clip:
            return self.adapter._clip_joint_deg(target_deg)
        return target_deg

    def _joint_index_map(self, incoming_names: List[str]) -> List[int]:
        if not incoming_names:
            return list(range(6))
        name_to_index = {name: index for index, name in enumerate(incoming_names)}
        return [name_to_index.get(name, -1) for name in self.mapping.joint_names]

    def _estimate_motor_speeds_rpm(
        self,
        previous_motor_deg: List[float],
        target_motor_deg: List[float],
        interval_ms: int,
    ) -> List[float]:
        dt_s = max(float(interval_ms) / 1000.0, 0.001)
        speeds = []
        for index, target in enumerate(target_motor_deg):
            delta_deg = abs(float(target) - float(previous_motor_deg[index]))
            rpm = max(3.0, min((delta_deg / dt_s) / 6.0, 500.0))
            speeds.append(rpm)
        return speeds

    def _estimate_single_point_speed_rpm(
        self,
        target_joint_deg: List[float],
        point: JointTrajectoryPoint,
        current_joint_deg: List[float],
    ) -> float:
        duration_s = max(
            float(point.time_from_start.sec)
            + float(point.time_from_start.nanosec) / 1_000_000_000.0,
            0.2,
        )
        max_motor_delta = 0.0
        for index, target in enumerate(target_joint_deg):
            current_motor = self.adapter.joint_deg_to_motor_deg(index, current_joint_deg[index])
            target_motor = self.adapter.joint_deg_to_motor_deg(index, target)
            max_motor_delta = max(max_motor_delta, abs(target_motor - current_motor))
        return max(3.0, min((max_motor_delta / duration_s) / 6.0, 300.0))

    def _on_display_planned_path(self, msg: DisplayTrajectory) -> None:
        if not self._debug_logging_enabled() or not msg.trajectory:
            return

        joint_trajectory = msg.trajectory[-1].joint_trajectory
        if not joint_trajectory.points:
            return

        start_deg = self._point_to_joint_deg(
            joint_trajectory.joint_names,
            joint_trajectory.points[0],
            fallback_deg=[0.0] * 6,
            clip=False,
        )
        final_deg = self._point_to_joint_deg(
            joint_trajectory.joint_names,
            joint_trajectory.points[-1],
            fallback_deg=start_deg,
            clip=False,
        )
        self._last_plan_final_deg = list(final_deg)
        self._last_plan_point_count = len(joint_trajectory.points)
        self._last_plan_stamp_sec = time.time()

        self.get_logger().info(
            "MoveIt plan debug: "
            f"points={self._last_plan_point_count}, "
            f"start_deg={self._format_deg(start_deg)}, "
            f"final_deg={self._format_deg(final_deg)}, "
            f"delta_deg={self._format_deg(self._subtract_deg(final_deg, start_deg))}"
        )

    def _log_execute_request(
        self,
        joint_names: List[str],
        points: List[JointTrajectoryPoint],
        start_deg: List[float],
        requested_final_deg: List[float],
        clipped_final_deg: List[float],
    ) -> None:
        if not self._debug_logging_enabled():
            return

        final_time_s = self._point_time_s(points[-1])
        move_delta_deg = self._subtract_deg(clipped_final_deg, start_deg)
        self.get_logger().info(
            "Execute request debug: "
            f"points={len(points)}, final_time_s={final_time_s:.3f}, "
            f"incoming_joints={joint_names or self.mapping.joint_names}, "
            f"current_deg={self._format_deg(start_deg)}, "
            f"requested_final_deg={self._format_deg(requested_final_deg)}, "
            f"clipped_final_deg={self._format_deg(clipped_final_deg)}, "
            f"move_delta_deg={self._format_deg(move_delta_deg)}"
        )

        if self._last_plan_final_deg is None:
            return

        age_s = time.time() - self._last_plan_stamp_sec
        plan_delta = self._subtract_deg(clipped_final_deg, self._last_plan_final_deg)
        max_plan_delta = max(abs(value) for value in plan_delta)
        log = self.get_logger().info if max_plan_delta <= 0.5 else self.get_logger().warning
        log(
            "Plan-vs-execute debug: "
            f"plan_age_s={age_s:.2f}, plan_points={self._last_plan_point_count}, "
            f"plan_final_deg={self._format_deg(self._last_plan_final_deg)}, "
            f"execute_final_deg={self._format_deg(clipped_final_deg)}, "
            f"diff_deg={self._format_deg(plan_delta)}"
        )

    def _log_single_point_command(self, final_deg: List[float], speed_rpm: float) -> None:
        if not self._debug_logging_enabled():
            return
        motor_deg = [
            self.adapter.joint_deg_to_motor_deg(index, final_deg[index])
            for index in range(6)
        ]
        self.get_logger().info(
            "Single-point command debug: "
            f"target_joint_deg={self._format_deg(final_deg)}, "
            f"target_motor_deg={self._format_deg(motor_deg)}, "
            f"speed_rpm={speed_rpm:.2f}"
        )

    def _log_trajectory_command(self, sdk_points: List[dict]) -> None:
        if not self._debug_logging_enabled() or not sdk_points:
            return
        final_motor_deg = [float(value) for value in sdk_points[-1]["positions"]]
        final_joint_deg = [
            self.adapter.motor_deg_to_joint_deg(index, final_motor_deg[index])
            for index in range(6)
        ]
        final_speeds = [float(value) for value in sdk_points[-1]["speeds"]]
        total_time_ms = sum(int(point["interval_ms"]) for point in sdk_points)
        self.get_logger().info(
            "Trajectory upload debug: "
            f"sdk_points={len(sdk_points)}, total_time_s={total_time_ms / 1000.0:.3f}, "
            f"final_joint_deg={self._format_deg(final_joint_deg)}, "
            f"final_motor_deg={self._format_deg(final_motor_deg)}, "
            f"final_speed_rpm={self._format_deg(final_speeds)}"
        )

    def _wait_for_goal_reached(
        self,
        target_deg: List[float],
        final_point: JointTrajectoryPoint,
    ) -> tuple[List[float], List[float], bool]:
        tolerance = max(0.0, float(self.get_parameter("execution_goal_tolerance_deg").value))
        period_s = max(0.05, float(self.get_parameter("execution_state_check_period_sec").value))
        settle_s = max(0.0, float(self.get_parameter("execution_settle_timeout_sec").value))
        timeout_s = max(settle_s, self._point_time_s(final_point) + settle_s)
        deadline = time.time() + timeout_s

        actual_deg = self.adapter.read_joint_positions_deg(force=True)
        error_deg = self._subtract_deg(target_deg, actual_deg)
        best_abs_error = max(abs(value) for value in error_deg)
        had_clean_sample = not self.adapter.last_read_had_error

        while time.time() < deadline:
            actual_deg = self.adapter.read_joint_positions_deg(force=True)
            if self.adapter.last_read_had_error:
                time.sleep(period_s)
                continue
            had_clean_sample = True
            error_deg = self._subtract_deg(target_deg, actual_deg)
            max_abs_error = max(abs(value) for value in error_deg)
            best_abs_error = min(best_abs_error, max_abs_error)
            if max_abs_error <= tolerance:
                if self._debug_logging_enabled():
                    self.get_logger().info(
                        "Goal reached debug: "
                        f"target_deg={self._format_deg(target_deg)}, "
                        f"actual_deg={self._format_deg(actual_deg)}, "
                        f"error_deg={self._format_deg(error_deg)}, "
                        f"tolerance_deg={tolerance:.2f}"
                    )
                return actual_deg, error_deg, True
            time.sleep(period_s)

        self.get_logger().warning(
            "Goal not reached debug: "
            f"target_deg={self._format_deg(target_deg)}, "
            f"actual_deg={self._format_deg(actual_deg)}, "
            f"error_deg={self._format_deg(error_deg)}, "
            f"best_max_error_deg={best_abs_error:.2f}, "
            f"had_clean_sample={had_clean_sample}, "
            f"tolerance_deg={tolerance:.2f}, timeout_s={timeout_s:.2f}"
        )
        return actual_deg, error_deg, False

    def _stop_after_execute_error(self, reason: str) -> None:
        if not bool(self.get_parameter("emergency_stop_on_execute_error").value):
            return
        try:
            self.adapter.emergency_stop()
            self.get_logger().warning(f"Emergency stop requested after execute error: {reason}")
        except Exception as exc:
            self.get_logger().warning(f"Emergency stop after execute error failed: {exc}")

    def _log_joint_mapping(self) -> None:
        if not self._debug_logging_enabled():
            return
        limits = [f"[{lo:.1f},{hi:.1f}]" for lo, hi in self.mapping.joint_limits_deg]
        entries = []
        for index, name in enumerate(self.mapping.joint_names):
            entries.append(
                f"{name}:motor={self.mapping.motor_ids[index]},"
                f"dir={self.mapping.directions[index]},"
                f"ratio={self.mapping.reducer_ratios[index]:.3f},"
                f"zero={self.mapping.zero_offsets_deg[index]:.3f},"
                f"limit={limits[index]}"
            )
        self.get_logger().info("Joint mapping debug: " + "; ".join(entries))

    def _debug_logging_enabled(self) -> bool:
        return bool(self.get_parameter("trajectory_debug_logging").value)

    def _point_time_s(self, point: JointTrajectoryPoint) -> float:
        return float(point.time_from_start.sec) + float(point.time_from_start.nanosec) / 1_000_000_000.0

    def _subtract_deg(self, left: List[float], right: List[float]) -> List[float]:
        return [float(left[index]) - float(right[index]) for index in range(6)]

    def _format_deg(self, values: List[float]) -> str:
        parts = []
        for index, value in enumerate(values[:6]):
            parts.append(f"J{index + 1}={float(value):+.2f}")
        return "[" + ", ".join(parts) + "]"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HorizonArmDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
