from dataclasses import dataclass

from revolve2.modular_robot.body import Module
from revolve2.simulation.scene import Pose, RigidBody


@dataclass
class UnbuiltChild:
    """A dataclass to store unbuilt children for the builders."""

    module: Module
    rigid_body: RigidBody
    pose: Pose
