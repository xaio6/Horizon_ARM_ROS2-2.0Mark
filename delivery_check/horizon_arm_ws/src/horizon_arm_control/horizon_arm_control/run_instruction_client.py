from __future__ import annotations

import rclpy
from horizon_arm_interfaces.action import RunInstruction
from rclpy.action import ActionClient
from rclpy.node import Node


class RunInstructionClient(Node):
    def __init__(self) -> None:
        super().__init__("horizon_arm_run_instruction_client")

        self.declare_parameter("instruction", "preset:home_position")
        self.declare_parameter("action_name", "/horizon_arm/run_instruction")

        self._instruction = str(self.get_parameter("instruction").value)
        self._client = ActionClient(
            self,
            RunInstruction,
            str(self.get_parameter("action_name").value),
        )
        self._sent = False
        self._timer = self.create_timer(0.5, self._send_once)

    def _send_once(self) -> None:
        if self._sent:
            return
        if not self._client.wait_for_server(timeout_sec=0.2):
            self.get_logger().info("Waiting for RunInstruction action server...")
            return
        goal = RunInstruction.Goal()
        goal.instruction = self._instruction
        self.get_logger().info(f"Sending instruction: {self._instruction}")
        self._sent = True
        future = self._client.send_goal_async(
            goal,
            feedback_callback=self._on_feedback,
        )
        future.add_done_callback(self._on_goal_response)

    def _on_feedback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f"[feedback] stage={feedback.stage} progress={feedback.progress:.2f} detail={feedback.detail}"
        )

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Instruction goal was rejected.")
            self._shutdown()
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Instruction succeeded: {result.message}")
        else:
            self.get_logger().error(f"Instruction failed: {result.message}")
        self._shutdown()

    def _shutdown(self) -> None:
        self.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RunInstructionClient()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
