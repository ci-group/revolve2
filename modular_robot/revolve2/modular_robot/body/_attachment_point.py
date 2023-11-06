from dataclasses import dataclass

from pyrr import Quaternion, Vector3


@dataclass
class AttachmentPoint:
    """An attachment point on the parent module for the child."""

    rotation: Quaternion
    offset: Vector3
