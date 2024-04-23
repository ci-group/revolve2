from dataclasses import dataclass

from pyrr import Quaternion, Vector3


@dataclass
class AttachmentPoint:
    """
    An attachment point is a theoretical location on the parent module for the child to be attached to.

    The attachment of a module to its parent is not considered to be a separate AttachmentPoint.
    This class simply is used for potential children to be placed on the correct positions of the current module.
    """

    orientation: Quaternion
    """The orientation of the attachment point on the module."""
    offset: Vector3
    """The offset for the attachment point, with respect to the center of the module."""
