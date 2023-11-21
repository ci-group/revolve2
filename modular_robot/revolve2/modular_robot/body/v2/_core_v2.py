from pyrr import Vector3, Quaternion
import math

from pyrr import Vector3

from .._right_angles import RightAngles
from ..base import Core
from ._attachment_face_core_v2 import AttachmentFaceCoreV2


class CoreV2(Core):
    """The core module of a modular robot."""

    _BATTERY_MASS = 0.39712  # in kg
    _FRAME_MASS = 1.0644  # in kg

    _horizontal_offset = 0.029  # The horizontal offset for attachment positions (in m).
    _vertical_offset = 0.032  # The vertical offset for attachment positions (in m).
    _attachment_faces: dict[int, AttachmentFaceCoreV2]

    def __init__(
        self,
        rotation: float | RightAngles,
        num_batteries: int = 1,
    ):
        """
        Initialize this object.

        :param rotation: The modules rotation.
        :param num_batteries: The amount of batteries in the robot.
        """
        mass = (
            num_batteries * self._BATTERY_MASS + self._FRAME_MASS
        )  # adjust if multiple batteries are installed

        """Here we produce the attachment faces, with the advanced logic behind conflict checking."""
        self._attachment_faces = {
            self.FRONT: AttachmentFaceCoreV2(
                horizontal_offset=self._horizontal_offset,
                vertical_offset=self._vertical_offset,
                face_rotation=0.0,
            ),
            self.BACK: AttachmentFaceCoreV2(
                horizontal_offset=self._horizontal_offset,
                vertical_offset=self._vertical_offset,
                face_rotation=math.pi,
            ),
            self.RIGHT: AttachmentFaceCoreV2(
                horizontal_offset=self._horizontal_offset,
                vertical_offset=self._vertical_offset,
                face_rotation=math.pi / 2.0,
            ),
            self.LEFT: AttachmentFaceCoreV2(
                horizontal_offset=self._horizontal_offset,
                vertical_offset=self._vertical_offset,
                face_rotation=math.pi / 2.0 * 3,
            ),
        }
        super().__init__(
            rotation=rotation,
            mass=mass,
            bounding_box=Vector3([0.15, 0.15, 0.15]),
            child_offset=0.0,
        )

        """Now we set the attachment faces as the children of the V2 Core."""
        self.front = self.attachment_faces[self.FRONT]
        self.back = self.attachment_faces[self.BACK]
        self.right = self.attachment_faces[self.RIGHT]
        self.left = self.attachment_faces[self.LEFT]

    @property
    def front_face(self) -> AttachmentFaceCoreV2:
        """
        Get the face attached to the front of the core.


        :returns: The attached module.
        """
        return self._attachment_faces[self.FRONT]

    @property
    def right_face(self) -> AttachmentFaceCoreV2:
        """
        Get the face attached to the right of the core.

        :returns: The attached module.
        """
        return self._attachment_faces[self.RIGHT]

    @property
    def back_face(self) -> AttachmentFaceCoreV2:
        """
        Get the face attached to the back of the core.

        :returns: The attached module.
        """
        return self._attachment_faces[self.BACK]

    @property
    def left_face(self) -> AttachmentFaceCoreV2:
        """
        Get the face attached to the left of the core.

        :returns: The attached module.
        """
        return self._attachment_faces[self.LEFT]

    @property
    def horizontal_offset(self) -> float:
        """
        Get the horizontal offset for attachment positions (in m).

        :return: The value.
        """
        return self._horizontal_offset

    @property
    def vertical_offset(self) -> float:
        """
        Get the vertical offset for attachment positions (in m).

        :return: The value.
        """
        return self._vertical_offset

    @property
    def attachment_faces(self) -> dict[int, AttachmentFaceCoreV2]:
        """
        Get all attachment faces for the Core.

        :return: The attachment faces.
        """
        return self._attachment_faces
