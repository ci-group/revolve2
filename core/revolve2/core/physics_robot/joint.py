from dataclasses import dataclass

from .rigid_body import RigidBody


@dataclass
class Joint:
    body1: RigidBody
    body2: RigidBody
