#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from moveit_msgs.msg import AttachedCollisionObject
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


class PlanningSceneSafetyObjects(Node):
    def __init__(self) -> None:
        super().__init__("horizon_arm_planning_scene_safety_objects")

        self.declare_parameter("world_frame", "world")
        self.declare_parameter("publish_period_sec", 2.0)
        self.declare_parameter("floor_enabled", True)
        self.declare_parameter("floor_z", -0.03)
        self.declare_parameter("floor_thickness", 0.06)
        self.declare_parameter("floor_size_x", 2.0)
        self.declare_parameter("floor_size_y", 2.0)
        self.declare_parameter("table_enabled", True)
        self.declare_parameter("table_z", -0.015)
        self.declare_parameter("table_thickness", 0.03)
        self.declare_parameter("table_size_x", 1.2)
        self.declare_parameter("table_size_y", 1.2)
        self.declare_parameter("workspace_box_enabled", False)
        self.declare_parameter("workspace_min_z", -0.2)
        self.declare_parameter("workspace_max_z", 0.6)
        self.declare_parameter("workspace_half_x", 0.6)
        self.declare_parameter("workspace_half_y", 0.6)
        self.declare_parameter("workspace_wall_thickness", 0.02)
        self.declare_parameter("workspace_ceiling_thickness", 0.02)
        self.declare_parameter("tool_collision_enabled", False)
        self.declare_parameter("tool_collision_link", "tool0")
        self.declare_parameter("tool_collision_size", [0.04, 0.04, 0.12])
        self.declare_parameter("tool_collision_center_offset", [0.0, 0.015, 0.06])

        self._publisher = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self._timer = self.create_timer(
            max(0.5, float(self.get_parameter("publish_period_sec").value)),
            self._publish_scene,
        )
        self._publish_scene()

    def _publish_scene(self) -> None:
        scene = PlanningScene()
        scene.is_diff = True

        if bool(self.get_parameter("floor_enabled").value):
            scene.world.collision_objects.append(
                self._box(
                    object_id="safety_floor",
                    size_x=float(self.get_parameter("floor_size_x").value),
                    size_y=float(self.get_parameter("floor_size_y").value),
                    size_z=float(self.get_parameter("floor_thickness").value),
                    center_z=float(self.get_parameter("floor_z").value)
                    - float(self.get_parameter("floor_thickness").value) / 2.0,
                )
            )

        if bool(self.get_parameter("table_enabled").value):
            scene.world.collision_objects.append(
                self._box(
                    object_id="safety_table",
                    size_x=float(self.get_parameter("table_size_x").value),
                    size_y=float(self.get_parameter("table_size_y").value),
                    size_z=float(self.get_parameter("table_thickness").value),
                    center_z=float(self.get_parameter("table_z").value)
                    - float(self.get_parameter("table_thickness").value) / 2.0,
                )
            )

        if bool(self.get_parameter("workspace_box_enabled").value):
            scene.world.collision_objects.extend(self._workspace_box_objects())

        if bool(self.get_parameter("tool_collision_enabled").value):
            scene.robot_state.attached_collision_objects.append(self._tool_collision_object())
            scene.robot_state.is_diff = True

        self._publisher.publish(scene)

    def _box(self, *, object_id: str, size_x: float, size_y: float, size_z: float, center_z: float) -> CollisionObject:
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size_x, size_y, size_z]

        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.z = center_z

        obj = CollisionObject()
        obj.header.frame_id = str(self.get_parameter("world_frame").value)
        obj.id = object_id
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        return obj

    def _workspace_box_objects(self) -> list[CollisionObject]:
        half_x = float(self.get_parameter("workspace_half_x").value)
        half_y = float(self.get_parameter("workspace_half_y").value)
        min_z = float(self.get_parameter("workspace_min_z").value)
        max_z = float(self.get_parameter("workspace_max_z").value)
        wall_t = float(self.get_parameter("workspace_wall_thickness").value)
        ceiling_t = float(self.get_parameter("workspace_ceiling_thickness").value)
        mid_z = (min_z + max_z) * 0.5
        height = max(0.05, max_z - min_z)

        objects = [
            self._box(
                object_id="workspace_wall_x_plus",
                size_x=wall_t,
                size_y=2.0 * half_y,
                size_z=height,
                center_z=mid_z,
            ),
            self._box(
                object_id="workspace_wall_x_minus",
                size_x=wall_t,
                size_y=2.0 * half_y,
                size_z=height,
                center_z=mid_z,
            ),
            self._box(
                object_id="workspace_wall_y_plus",
                size_x=2.0 * half_x,
                size_y=wall_t,
                size_z=height,
                center_z=mid_z,
            ),
            self._box(
                object_id="workspace_wall_y_minus",
                size_x=2.0 * half_x,
                size_y=wall_t,
                size_z=height,
                center_z=mid_z,
            ),
            self._box(
                object_id="workspace_ceiling",
                size_x=2.0 * half_x,
                size_y=2.0 * half_y,
                size_z=ceiling_t,
                center_z=max_z + ceiling_t * 0.5,
            ),
        ]
        objects[0].primitive_poses[0].position.x = half_x + wall_t * 0.5
        objects[1].primitive_poses[0].position.x = -half_x - wall_t * 0.5
        objects[2].primitive_poses[0].position.y = half_y + wall_t * 0.5
        objects[3].primitive_poses[0].position.y = -half_y - wall_t * 0.5
        return objects

    def _tool_collision_object(self) -> AttachedCollisionObject:
        size = [float(value) for value in self.get_parameter("tool_collision_size").value]
        center_offset = [
            float(value) for value in self.get_parameter("tool_collision_center_offset").value
        ]

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = size

        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = center_offset[0]
        pose.position.y = center_offset[1]
        pose.position.z = center_offset[2]

        tool_object = CollisionObject()
        tool_object.header.frame_id = str(self.get_parameter("tool_collision_link").value)
        tool_object.id = "tool_collision_box"
        tool_object.primitives.append(primitive)
        tool_object.primitive_poses.append(pose)
        tool_object.operation = CollisionObject.ADD

        attached = AttachedCollisionObject()
        attached.link_name = str(self.get_parameter("tool_collision_link").value)
        attached.object = tool_object
        return attached


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanningSceneSafetyObjects()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
