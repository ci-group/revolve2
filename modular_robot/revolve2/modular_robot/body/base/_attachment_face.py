from .._attachment_point import AttachmentPoint
from .._color import Color
from .._module import Module
from .._right_angles import RightAngles


class AttachmentFace(Module):
    """
    Collect AttachmentPoints on a modules face.

    This face can be thought of as a pseudo-module which usually does not have a body on its own.
    """

    def __init__(
        self,
        rotation: float | RightAngles,
        attachment_points: dict[int, AttachmentPoint],
    ) -> None:
        """
        Initialize this object.

        :param rotation: Orientation of this model relative to its parent.
        :param attachment_points: The attachment points available on a module.
        """
        super().__init__(
            rotation=rotation,
            attachment_points=attachment_points,
            color=Color(255, 255, 255, 255),
            sensors=[],
        )
