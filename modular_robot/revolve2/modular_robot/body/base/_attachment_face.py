from abc import abstractmethod
from dataclasses import dataclass, field

from .._attachment_point import AttachmentPoint
from .._module import Module


@dataclass
class AttachmentFace:

    """Collect AttachmentPoints on a modules face."""

    _attachment_points: dict[int, AttachmentPoint] = field(default_factory=lambda: {})
    _mounted_modules: dict[int, Module] = field(default_factory=lambda: {})

    def add_attachment_point(
        self,
        index: int,
        attachment_point: AttachmentPoint,
        allow_repopulation: bool = False,
    ) -> None:
        """
        Add an attachm<<<<<<< HEADent point to the Face.

        :param index: The index of the attachment point.
        :param attachment_point: The attachment point.
        :param allow_repopulation: If setting a new attachment point onto a previously existing one is allowed.
        """
        if not allow_repopulation:
            assert self._attachment_points.get(index) is None, "Index is already used."

        self._attachment_points[index] = attachment_point

    def mount_module(
        self, index: int, module: Module, ignore_conflict: bool = False
    ) -> bool:
        """
        Mount a Module onto a attachment point.

        :param index: The index of the attachment point.
        :param module: The module.
        :param ignore_conflict: Ignore potential errors when mounting to attachment_point.
        :return: If successful or not.
        """
        if not self._check_for_conflict(index, module, ignore_conflict):
            self._mounted_modules[index] = module
            return True
        return False

    @property
    def attachment_points(self) -> dict[int, AttachmentPoint]:
        """
        Get the attachment points.

        :return: The attachment points.
        """
        return self._attachment_points

    @abstractmethod
    def _check_for_conflict(
        self, index: int, module: Module, ignore_conflict: bool
    ) -> bool:
        """
        Check for conflicts when adding a new attachment point.

        :param index: The index of the attachment point.
        :param module: The module.
        :param ignore_conflict: Ignore potential errors when adding attachment_point.
        :raises Exception: If the attachment point causes conflicts.
        :return : If conflicts occurred.
        """
        pass
