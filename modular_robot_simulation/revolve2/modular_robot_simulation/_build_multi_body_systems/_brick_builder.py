import math

from pyrr import Quaternion, Vector3

from revolve2.modular_robot.body.base import Brick
from revolve2.simulation.scene import AABB, MultiBodySystem, Pose, RigidBody
from revolve2.simulation.scene.geometry import GeometryBox
from revolve2.simulation.scene.geometry.textures import Texture

from ._body_to_multi_body_system_mapping import BodyToMultiBodySystemMapping
from ._builder import Builder
from ._convert_color import convert_color
from ._unbuilt_child import UnbuiltChild


class BrickBuilder(Builder):
    """A Builder for Bricks."""

    _module: Brick

    def __init__(self, module: Brick, rigid_body: RigidBody, slot_pose: Pose):
        """
        Initialize the Brick Builder.

        :param module: The module to be built.
        :param rigid_body: The rigid body for the module to be built on.
        :param slot_pose: The slot pose of the module.
        """
        self._module = module
        self._rigid_body = rigid_body
        self._slot_pose = slot_pose

    def build(
        self,
        multi_body_system: MultiBodySystem,
        body_to_multi_body_system_mapping: BodyToMultiBodySystemMapping,
    ) -> list[UnbuiltChild]:
        """
        Build a module onto the Robot.

        :param multi_body_system: The multi body system of the robot.
        :param body_to_multi_body_system_mapping: A mapping from body to multi-body system
        :return: The next children to be built.
        """
        brick_center_pose = Pose(
            self._slot_pose.position
            + self._slot_pose.orientation
            * Vector3([self._module.bounding_box[0] / 2.0, 0.0, 0.0]),
            self._slot_pose.orientation,
        )

        self._rigid_body.geometries.append(
            GeometryBox(
                pose=brick_center_pose,
                mass=self._module.mass,
                texture=Texture(base_color=convert_color(self._module.color)),
                aabb=AABB(self._module.bounding_box),
            )
        )

        tasks = []
        for child_index, angle in [
            (self._module.FRONT, 0.0),
            (self._module.LEFT, math.pi / 2.0),
            (self._module.RIGHT, math.pi / 2.0 * 3),
        ]:
            child = self._module.children[child_index]
            if child is not None:
                child_slot_pose = Pose(
                    position=brick_center_pose.position
                    + brick_center_pose.orientation
                    * Quaternion.from_eulers([0.0, 0.0, angle])
                    * Vector3([self._module.child_offset, 0.0, 0.0]),
                    orientation=brick_center_pose.orientation
                    * Quaternion.from_eulers([0.0, 0.0, angle])
                    * Quaternion.from_eulers([child.rotation, 0, 0]),
                )

                tasks.append(
                    UnbuiltChild(
                        module=child,
                        rigid_body=self._rigid_body,
                        pose=child_slot_pose,
                    )
                )
        return tasks
