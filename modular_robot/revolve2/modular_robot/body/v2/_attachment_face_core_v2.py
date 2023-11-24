from dataclasses import dataclass, field
from itertools import product

import numpy as np
from numpy.typing import NDArray
from pyrr import Quaternion, Vector3

from .. import Module
from .._attachment_point import AttachmentPoint
from ..base._attachment_face import AttachmentFace


@dataclass
class AttachmentFaceCoreV2(AttachmentFace):
    """An AttachmentFace for the V2 robot."""

    _check_matrix: NDArray[np.uint8] = field(
        default_factory=lambda: np.zeros(shape=(3, 3), dtype=np.uint8)
    )

    _child_offset: Vector3 = field(
        default_factory=lambda: Vector3([0.15 / 2.0, 0.0, 0.0])
    )
    _orientation: Quaternion = field(default_factory=lambda: Quaternion())

    """
    Check matrix allows us to determine which attachment points can be filled in the face.
    
    check_matrix =  0   0   0
                      C   C  
                    0   0   0
                      C   C  
                    0   0   0
     
    By default the whole matrix is 0. Once we add a module at location x we adjust the C accordingly. 
    When adding a new module we want to have a C of 0 in the corresponding position, otherwise the attachment point cant be occupied anymore.
    Applying a simple 2D convolution allows fast conflict checks.
    """

    def __init__(
        self, orientation: Quaternion, horizontal_offset: float, vertical_offset: float
    ) -> None:
        """
        Initialize n attachment face for the V2 Core.

        :param orientation: The rotation of the face and the attachment points.
        :param horizontal_offset: The horizontal offset for module placement.
        :param vertical_offset:  The vertical offset for module placement.
        """
        self._orientation = orientation

        """
        Each CoreV2 Face has 9 Module slots as shown below. 
        
        ---------------------------------------------------
        |                 |            |                  |
        | 0 (TOP_LEFT)    | 1 (TOP)    | 2 (TOP_RIGHT)    |
        |                 |            |                  |
        | 3 (MIDDLE_LEFT) | 4 (MIDDLE) | 5 (MIDDLE_RIGHT) |
        |                 |            |                  |
        | 6 (BOTTOM_LEFT) | 7 (BOTTOM) | 8 (BOTTOM_RIGHT) |
        |                 |            |                  |
        ---------------------------------------------------
        """
        super().__init__()
        for i in range(9):
            h_o = (i % 3 - 1) * horizontal_offset
            v_o = -(i // 3 - 1) * vertical_offset

            sc: bool = orientation.angle % np.pi == 0
            """Switch condition for determining offset. Depending of whether the face is orthogonal to x or y axis, the horizontal offset has to be applied differently."""
            offset = Vector3([(1 - sc) * (-h_o), sc * h_o, v_o])

            attachment_point = AttachmentPoint(
                orientation=orientation, offset=self._child_offset + offset
            )
            self.add_attachment_point(i, attachment_point)

    def _check_for_conflict(
        self, index: int, module: Module, ignore_conflict: bool
    ) -> bool:
        """
        Check for conflicts when adding a new attachment point.

        :param index: The index of the attachment point.
        :param module: The module.
        :param ignore_conflict: Ignore potential conflicts when mounting to attachment_point.
        :return: Whether conflicts occurred.
        :raises Exception: If the attachment point causes conflicts.
        """
        check_matrix = self._check_matrix.copy()
        check_matrix[(index - 1) % 3, (index - 1) // 3] += 1
        conv_check = np.zeros(shape=(2, 2), dtype=np.uint8)
        for i, j in product(range(2), repeat=2):
            conv_check[i, j] = np.sum(check_matrix[i : i + 1, j : j + 1])

        if np.max(conv_check) > 1:  # Conflict detected.
            if ignore_conflict:
                return True
            raise Exception("CONFLICT: Module can`t be attached at the AttachmentPoint")
        self._check_matrix = check_matrix
        return False

    @property
    def top_left(self) -> Module | None:
        """
        Get the top_left attachment points module.

        :return: The attachment points module.
        """
        return self._mounted_modules.get(0)

    @top_left.setter
    def top_left(self, module: Module) -> None:
        """
        Set a module to the top_left attachment point.

        :param module: The module.
        """
        self.mount_module(0, module)

    @property
    def top(self) -> Module | None:
        """
        Get the top attachment points module.

        :return: The attachment points module.
        """
        return self._mounted_modules.get(1)

    @top.setter
    def top(self, module: Module) -> None:
        """
        Set a module to the top attachment point.

        :param module: The module.
        """
        self.mount_module(1, module)

    @property
    def top_right(self) -> Module | None:
        """
        Get the top_right attachment points module.

        :return: The attachment points module.
        """
        return self._mounted_modules.get(2)

    @top_right.setter
    def top_right(self, module: Module) -> None:
        """
        Set a module to the top_right attachment point.

        :param module: The module.
        """
        self.mount_module(2, module)

    @property
    def middle_left(self) -> Module | None:
        """
        Get the middle_left attachment points module.

        :return: The attachment points module.
        """
        return self._mounted_modules.get(3)

    @middle_left.setter
    def middle_left(self, module: Module) -> None:
        """
        Set a module to the middle_left attachment point.

        :param module: The module.
        """
        self.mount_module(3, module)

    @property
    def middle(self) -> Module | None:
        """
        Get the middle attachment points module.

        :return: The attachment points module.
        """
        return self._mounted_modules.get(4)

    @middle.setter
    def middle(self, module: Module) -> None:
        """
        Set a module to the middle attachment point.

        :param module: The module.
        """
        self.mount_module(4, module)

    @property
    def middle_right(self) -> Module | None:
        """
        Get the middle_right attachment points module.

        :return: The attachment points module.
        """
        return self._mounted_modules.get(5)

    @middle_right.setter
    def middle_right(self, module: Module) -> None:
        """
        Set a module to the middle_right attachment point.

        :param module: The module.
        """
        self.mount_module(5, module)

    @property
    def bottom_left(self) -> Module | None:
        """
        Get the bottom_left attachment points module.

        :return: The attachment points module.
        """
        return self._mounted_modules.get(6)

    @bottom_left.setter
    def bottom_left(self, module: Module) -> None:
        """
        Set a module to the bottom_left attachment point.

        :param module: The module.
        """
        self.mount_module(6, module)

    @property
    def bottom(self) -> Module | None:
        """
        Get the bottom attachment points module.

        :return: The attachment points module.
        """
        return self._mounted_modules.get(7)

    @bottom.setter
    def bottom(self, module: Module) -> None:
        """
        Set a module to the bottom attachment point.

        :param module: The module.
        """
        self.mount_module(7, module)

    @property
    def bottom_right(self) -> Module | None:
        """
        Get the bottom_right attachment points module.

        :return: The attachment points module.
        """
        return self._mounted_modules.get(8)

    @bottom_right.setter
    def bottom_right(self, module: Module) -> None:
        """
        Set a module to the bottom_right attachment point.

        :param module: The module.
        """
        self.mount_module(8, module)
