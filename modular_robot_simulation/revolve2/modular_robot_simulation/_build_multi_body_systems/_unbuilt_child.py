from dataclasses import dataclass, field

from pyrr import Quaternion, Vector3
from revolve2.modular_robot.body import Module
from revolve2.modular_robot.body.sensors import Sensor
from revolve2.simulation.scene import Pose, RigidBody


@dataclass
class UnbuiltChild:
    """A dataclass to store unbuilt children for the builders."""

    child_object: Module | Sensor
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
        self.pose = Pose(
            position,
            orientation * self.child_object.orientation,
        )
