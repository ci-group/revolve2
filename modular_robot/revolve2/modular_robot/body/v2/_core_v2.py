import math

from pyrr import Quaternion, Vector3

from .._color import Color
from .._right_angles import RightAngles
from ..base import Core
from ._attachment_face_core_v2 import AttachmentFaceCoreV2


class CoreV2(Core):
    """The core module of a modular robot."""

    _BATTERY_MASS = 0.39712  # in kg
    _FRAME_MASS = 1.0644  # in kg
    _COLOR = Color(255, 50, 50, 255)

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

        self._attachment_faces = {
            self.FRONT: AttachmentFaceCoreV2(
                Quaternion.from_eulers([0.0, 0.0, 0.0]),
                self._horizontal_offset,
                self._vertical_offset,
            ),
            self.BACK: AttachmentFaceCoreV2(
                Quaternion.from_eulers([0.0, 0.0, math.pi]),
                self._horizontal_offset,
                self._vertical_offset,
            ),
            self.LEFT: AttachmentFaceCoreV2(
                Quaternion.from_eulers([0.0, 0.0, math.pi / 2.0]),
                self._horizontal_offset,
                self._vertical_offset,
            ),
            self.RIGHT: AttachmentFaceCoreV2(
                Quaternion.from_eulers([0.0, 0.0, math.pi / 2.0 * 3]),
                self._horizontal_offset,
                self._vertical_offset,
            ),
        }

        attachment_points = {}
        for index, face in self._attachment_faces.items():
            for idx, point in face.attachment_points.items():
                new_index = self.index_from_face_and_attachment(index, idx)
                attachment_points[new_index] = point

        super().__init__(
            rotation=rotation,
            color=self._COLOR,
            mass=mass,
            bounding_box=Vector3([0.15, 0.15, 0.15]),
            attachment_points=attachment_points,
        )

    @property
    def front(self) -> AttachmentFaceCoreV2:
        """
        Get the module attached to the front of the core.

        :returns: The attached module.
        """
        return self._attachment_faces[self.FRONT]

    @property
    def right(self) -> AttachmentFaceCoreV2:
        """
        Get the module attached to the right of the core.

        :returns: The attached module.
        """
        return self._attachment_faces[self.RIGHT]

    @property
    def back(self) -> AttachmentFaceCoreV2:
        """
        Get the module attached to the back of the core.

        :returns: The attached module.
        """
        return self._attachment_faces[self.BACK]

    @property
    def left(self) -> AttachmentFaceCoreV2:
        """
        Get the module attached to the left of the core.

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

    @staticmethod
    def index_from_face_and_attachment(face_index: int, attachment_index: int) -> int:
        """
        Generate global indices specific for the V2 Core. Formats the new index as such: <face_index>0<point_index>.

        :param face_index: The index of the face.
        :param attachment_index: The index of the attachment position.
        :return: The global index.
        """
        global_index = (
            face_index * 10 ** (1 + len(str(attachment_index))) + attachment_index
        )
        return int(global_index)
