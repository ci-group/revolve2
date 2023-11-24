from pyrr import Vector3

from ...body import Module
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

        super().__init__(
            rotation=rotation,
            mass=mass,
            bounding_box=Vector3([0.15, 0.15, 0.15]),
            child_offset=Vector3(
                [0.0, 0.0, 0.0]
            ),  # We initialize the core with placeholder Attachment Points.
        )

        """Now we produce the actual attachment points, with the advanced logic behind attachment faces."""
        self._attachment_faces = {
            side: AttachmentFaceCoreV2(
                placeholder_point.rotation,
                self._horizontal_offset,
                self._vertical_offset,
            )
            for side, placeholder_point in self.attachment_points.items()
        }

        new_attachment_points = {}
        for index, face in self._attachment_faces.items():
            for idx, point in face.attachment_points.items():
                new_index = self.index_from_face_and_attachment(index, idx)
                new_attachment_points[new_index] = point
        self._attachment_points = new_attachment_points

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
        global_index = "1" * face_index + "0" + "1" * attachment_index
        return int(global_index)

    @staticmethod
    def face_and_attachment_from_index(global_index: int) -> tuple[int, int]:
        """
        Get face and attachment indices from global index.

        :param global_index: The global index.
        :return: The face and attachment index.
        """
        head, tail = str(global_index).split("0")
        return len(head), len(tail)

    @property
    def attachment_faces(self) -> dict[int, AttachmentFaceCoreV2]:
        """
        Get all attachment faces for the Core.

        :return: The attachment faces.
        """
        return self._attachment_faces

    def set_child(self, module: Module, child_index: int) -> None:
        """
        Attach a module to a slot.

        :param module: The module to attach.
        :param child_index: The slot to attach it to.
        :raises KeyError: If attachment point is already populated.
        """
        assert (
            module._parent is None
        ), "Child module already connected to a different slot."
        module._parent = self
        module._parent_child_index = child_index
        if self.is_free(child_index) and self.can_set_child(module, child_index):
            self._children[child_index] = module
        else:
            raise KeyError("Attachment point already populated")

    def can_set_child(self, module: Module, child_index: int) -> bool:
        """
        Check if attaching child is allowed or has conflicts.

        :param module: The module to set.
        :param child_index: The childs global index.
        :return: Whether it is legal.
        """
        face_index, attachment_index = self.face_and_attachment_from_index(child_index)
        return self._attachment_faces[face_index].mount_module(
            attachment_index, module, ignore_conflict=True
        )
