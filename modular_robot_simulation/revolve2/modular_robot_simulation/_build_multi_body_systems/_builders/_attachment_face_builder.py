from revolve2.modular_robot.body.base import AttachmentFace
from revolve2.simulation.scene import MultiBodySystem, Pose, RigidBody

from .._body_to_multi_body_system_mapping import BodyToMultiBodySystemMapping
from .._unbuilt_child import UnbuiltChild
from ._builder import Builder


class AttachmentFaceBuilder(Builder):
    """A Builder for Attachment Faces."""

    _module: AttachmentFace

    def __init__(self, module: AttachmentFace, rigid_body: RigidBody, slot_pose: Pose):
        """
        Initialize the Attachment Face Builder.

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
        tasks = []
        for child_index, attachment_point in self._module.attachment_points.items():
            child = self._module.children.get(child_index)
            if child is not None:
                unbuilt = UnbuiltChild(
                    child_object=child,
                    rigid_body=self._rigid_body,
                )
                unbuilt.make_pose(
                    position=self._slot_pose.position
                    + self._slot_pose.orientation * attachment_point.offset,
                    orientation=self._slot_pose.orientation,
                )
                tasks.append(unbuilt)
        return tasks
