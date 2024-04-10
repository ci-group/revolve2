from pyrr import Quaternion, Vector3

from .._attachment_point import AttachmentPoint
from .._color import Color
from .._module import Module


class DroneCore(Module):
    """The core module of a modular robot."""

    def __init__(
        self,
        mass: float,
        bounding_box: Vector3,
    ) -> None:
        """
        Initialize this object.

        :param mass: The Modules mass (in kg).
        :param bounding_box: The bounding box. Vector3 with sizes of bbox in x,y,z dimension (m). Sizes are total length, not half length from origin.
        """
        self._mass = mass
        self._bounding_box = bounding_box
        # Attachment points are variable in this case. So a mechanism for continually adding attachment
        # points needs to be in place
        self.num_attachments = 0

        ## TODO: Remove rotation from inherited module class
        super().__init__(0.0, Color(255, 50, 50, 255), {}, [])

    @property
    def mass(self) -> float:
        """
        Return the mass of the core.

        :return: mass of the Core (in kg).
        """
        return self._mass

    @property
    def bounding_box(self) -> Vector3:
        """
        Get the bounding box.

        Sizes are total length, not half length from origin.
        :return: Vector3 with sizes of bbox in x,y,z dimension (m).
        """
        return self._bounding_box

    def add_attachment(
        self,
        module: Module,
        attachment_point: AttachmentPoint = AttachmentPoint(
            offset=Vector3([0, 0, 0]), orientation=Quaternion()
        ),
    ) -> None:
        """
        Add an attachment to the core and sets it as a child.

        :param module: The module to attach.
        :param attachment_point: Optional argument specifing where to attachmentpoint is. Default is modules pose.
        """
        self.set_child(module, self.num_attachments)
        self.attachment_points.update({self.num_attachments: attachment_point})
        self.num_attachments += 1
