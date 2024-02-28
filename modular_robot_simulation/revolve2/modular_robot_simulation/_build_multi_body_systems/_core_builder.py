from revolve2.modular_robot.body.base import Core
from revolve2.simulation.scene import AABB, MultiBodySystem, Pose, RigidBody
from revolve2.simulation.scene.geometry import GeometryBox
from revolve2.simulation.scene.geometry.textures import Texture

from ._body_to_multi_body_system_mapping import BodyToMultiBodySystemMapping
from ._builder import Builder
from ._convert_color import convert_color
from ._unbuilt_child import UnbuiltChild


class CoreBuilder(Builder):
    """A Builder for Cores."""

    _module: Core

    def __init__(self, module: Core, rigid_body: RigidBody, slot_pose: Pose):
        """
        Initialize the Core V1 Builder.

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
        self._rigid_body.geometries.append(
            GeometryBox(
                pose=self._slot_pose,
                mass=self._module.mass,
                texture=Texture(base_color=convert_color(self._module.color)),
                aabb=AABB(self._module.bounding_box),
            )
        )

        tasks = []
        for child_index, attachment_point in self._module.attachment_points.items():
            child = self._module.children.get(child_index)
            if child is not None:
                unbuilt = UnbuiltChild(
                    module=child,
                    rigid_body=self._rigid_body,
                )
                unbuilt.make_pose(
                    position=self._slot_pose.position
                    + self._slot_pose.orientation
                    * attachment_point.orientation
                    * attachment_point.offset,
                    orientation=self._slot_pose.orientation
                    * attachment_point.orientation,
                )

                tasks.append(unbuilt)
        return tasks
