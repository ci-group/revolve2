import logging
from collections import deque

from pyrr import Quaternion, Vector3
from revolve2.modular_robot.body.base import Body
from revolve2.simulation.scene import MultiBodySystem, Pose, RigidBody

from ._body_to_multi_body_system_mapping import BodyToMultiBodySystemMapping
from ._get_builder import get_builder
from ._unbuilt_child import UnbuiltChild


class BodyToMultiBodySystemConverter:
    """A tool to convert modular robot bodies to multi-body systems."""

    _STATIC_FRICTION = 1.0
    _DYNAMIC_FRICTION = 1.0

    def convert_robot_body(
        self, body: Body, pose: Pose, translate_z_aabb: bool
    ) -> tuple[MultiBodySystem, BodyToMultiBodySystemMapping]:
        """
        Convert a modular robot body to a multi-body system.

        :param body: The body to convert.
        :param pose: The pose to put the multi-body system in. Ownership will be taken of the pose object and it should not be reused after passing it to this function.
        :param translate_z_aabb: Whether the robot should be translated upwards so it's T-pose axis-aligned bounding box is exactly on the ground. I.e. if the robot should be placed exactly on the ground. The pose parameters is still added afterwards.
        :returns: The created multi-body system, and a mapping from body to multi-body system.
        """
        multi_body_system = MultiBodySystem(pose=pose, is_static=False)

        rigid_body = RigidBody(
            initial_pose=Pose(),
            static_friction=self._STATIC_FRICTION,
            dynamic_friction=self._DYNAMIC_FRICTION,
            geometries=[],
        )

        mapping = BodyToMultiBodySystemMapping(multi_body_system=multi_body_system)

        multi_body_system.add_rigid_body(rigid_body)

        unbuilt = UnbuiltChild(
            child_object=body.core,
            rigid_body=rigid_body,
        )
        unbuilt.make_pose(Vector3())
        queue = deque([unbuilt])
        while len(queue) > 0:
            builder = get_builder(queue.popleft())
            new_tasks = builder.build(
                multi_body_system=multi_body_system,
                body_to_multi_body_system_mapping=mapping,
            )
            queue.extend(new_tasks)

        if translate_z_aabb:
            if pose.orientation != Quaternion():
                logging.info(
                    "translate_z_aabb does not yet support non-identity orientation. Orientation ignored for AABB calculation. Robot is probably not positioned as you would like."
                )
            aabb_position, aabb = multi_body_system.calculate_aabb()
            pose.position += Vector3([0.0, 0.0, -aabb_position.z + aabb.size.z / 2.0])

        return multi_body_system, mapping
