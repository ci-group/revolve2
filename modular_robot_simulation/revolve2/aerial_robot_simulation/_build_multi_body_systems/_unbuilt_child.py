from dataclasses import dataclass, field

from pyrr import Quaternion, Vector3

from revolve2.aerial_robot.body import Module
from revolve2.simulation.scene import Pose, RigidBody


@dataclass
class UnbuiltChild:
    """A dataclass to store unbuilt children for the builders."""

    module: Module
    rigid_body: RigidBody
    pose: Pose = field(init=False)

    def make_pose(
        self, position: Vector3, orientation: Quaternion = Quaternion()
    ) -> None:
        """
        Make the pose of the unbuilt child.

        :param position: The position argument from the parent.
        :param orientation: The orientation of the attachment on the parent.
        """
        module_rot = Quaternion.from_eulers([self.module.rotation, 0.0, 0.0])
        self.pose = Pose(
            position,
            orientation * module_rot,
        )
