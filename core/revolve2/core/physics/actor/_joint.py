from dataclasses import dataclass

from pyrr import Quaternion, Vector3

from ._rigid_body import RigidBody


@dataclass
class Joint:
    name: str
    body1: RigidBody
    body2: RigidBody
    position: Vector3
    orientation: Quaternion
    axis: Vector3
