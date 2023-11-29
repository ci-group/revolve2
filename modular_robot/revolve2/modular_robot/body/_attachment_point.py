from dataclasses import dataclass

from pyrr import Quaternion, Vector3


@dataclass
class AttachmentPoint:
    """An attachment point on the parent module for the child."""

    orientation: Quaternion
    """The orientation of the attachment point on the module."""
    offset: Vector3
    """The offset for the attachment point, with respect to the center of the module."""
