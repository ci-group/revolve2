from itertools import product

import numpy as np
from numpy.typing import NDArray
from pyrr import Quaternion, Vector3

from .. import Module
from .._attachment_point import AttachmentPoint
from ..base._attachment_face import AttachmentFace


class AttachmentFaceCoreV2(AttachmentFace):
    """An AttachmentFace for the V2 Core."""

    _check_matrix: NDArray[np.uint8]
    _child_offset: Vector3

    """
    Check matrix allows us to determine which attachment points can be filled in the face.
    
    check_matrix =  0   0   0
                      C   C  
                    0   0   0
                      C   C  
                    0   0   0
     
    By default the whole matrix is 0. Once we add a module at location x we adjust the C accordingly. 
    When adding a new module we want to have a C of 0 in the corresponding position, otherwise the attachment point cant be populated anymore.
    Applying a simple 2D convolution allows for fast conflict checks.
    """

    def __init__(
        self, face_rotation: float, horizontal_offset: float, vertical_offset: float
    ) -> None:
        """
        Initialize the attachment face for the V2 Core.

        :param face_rotation: The rotation of the face and the attachment points on the module.
        :param horizontal_offset: The horizontal offset for module placement.
        :param vertical_offset:  The vertical offset for module placement.
        """
        self._child_offset = Vector3([0.15 / 2.0, 0.0, 0.0])
        self._check_matrix = np.zeros(shape=(3, 3), dtype=np.uint8)

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
        attachment_points = {}
        rot = Quaternion.from_eulers([0.0, 0.0, face_rotation])
        for i in range(9):
            h_o = (i % 3 - 1) * horizontal_offset
            v_o = -(i // 3 - 1) * vertical_offset

            h_o = h_o if int(rot.angle / np.pi) % 2 == 0 else -h_o
            offset = Vector3([0.0, h_o, v_o]) if np.isclose(rot.angle % np.pi, 0) else Vector3([h_o, 0.0, v_o])
            offset = rot * offset

            attachment_points[i] = AttachmentPoint(
                orientation=rot, offset=self._child_offset + offset
            )
        super().__init__(rotation=0.0, attachment_points=attachment_points)

    def can_set_child(
        self,
        module: Module,
        child_index: int,
    ) -> bool:
        """
        Check for conflicts when adding a new attachment point.

        Note that if there is no conflict in the check this function assumes that the slot is being populated and adjusts the check-matrix as such.

        :param module: The module.
        :param child_index: The index of the attachment point.
        :return: Whether conflicts occurred.
        """
        check_matrix = self._check_matrix.copy()
        check_matrix[(child_index - 1) % 3, (child_index - 1) // 3] += 1
        conv_check = np.zeros(shape=(2, 2), dtype=np.uint8)
        for i, j in product(range(2), repeat=2):
            conv_check[i, j] = np.sum(check_matrix[i : i + 1, j : j + 1])

        if np.max(conv_check) > 1:  # Conflict detected.
            return False
        self._check_matrix = check_matrix
        return True

    @property
    def top_left(self) -> Module | None:
        """
        Get the top_left attachment points module.

        :return: The attachment points module.
        """
        return self.children.get(0)

    @top_left.setter
    def top_left(self, module: Module) -> None:
        """
        Set a module to the top_left attachment point.

        :param module: The module.
        """
        self.set_child(module, 0)

    @property
    def top(self) -> Module | None:
        """
        Get the top attachment points module.

        :return: The attachment points module.
        """
        return self.children.get(1)

    @top.setter
    def top(self, module: Module) -> None:
        """
        Set a module to the top attachment point.

        :param module: The module.
        """
        self.set_child(module, 1)

    @property
    def top_right(self) -> Module | None:
        """
        Get the top_right attachment points module.

        :return: The attachment points module.
        """
        return self.children.get(2)

    @top_right.setter
    def top_right(self, module: Module) -> None:
        """
        Set a module to the top_right attachment point.

        :param module: The module.
        """
        self.set_child(module, 2)

    @property
    def middle_left(self) -> Module | None:
        """
        Get the middle_left attachment points module.

        :return: The attachment points module.
        """
        return self.children.get(3)

    @middle_left.setter
    def middle_left(self, module: Module) -> None:
        """
        Set a module to the middle_left attachment point.

        :param module: The module.
        """
        self.set_child(module, 3)

    @property
    def middle(self) -> Module | None:
        """
        Get the middle attachment points module.

        :return: The attachment points module.
        """
        return self.children.get(4)

    @middle.setter
    def middle(self, module: Module) -> None:
        """
        Set a module to the middle attachment point.

        :param module: The module.
        """
        self.set_child(module, 4)

    @property
    def middle_right(self) -> Module | None:
        """
        Get the middle_right attachment points module.

        :return: The attachment points module.
        """
        return self.children.get(5)

    @middle_right.setter
    def middle_right(self, module: Module) -> None:
        """
        Set a module to the middle_right attachment point.

        :param module: The module.
        """
        self.set_child(module, 5)

    @property
    def bottom_left(self) -> Module | None:
        """
        Get the bottom_left attachment points module.

        :return: The attachment points module.
        """
        return self.children.get(6)

    @bottom_left.setter
    def bottom_left(self, module: Module) -> None:
        """
        Set a module to the bottom_left attachment point.

        :param module: The module.
        """
        self.set_child(module, 6)

    @property
    def bottom(self) -> Module | None:
        """
        Get the bottom attachment points module.

        :return: The attachment points module.
        """
        return self.children.get(7)

    @bottom.setter
    def bottom(self, module: Module) -> None:
        """
        Set a module to the bottom attachment point.

        :param module: The module.
        """
        self.set_child(module, 7)

    @property
    def bottom_right(self) -> Module | None:
        """
        Get the bottom_right attachment points module.

        :return: The attachment points module.
        """
        return self.children.get(8)

    @bottom_right.setter
    def bottom_right(self, module: Module) -> None:
        """
        Set a module to the bottom_right attachment point.

        :param module: The module.
        """
        self.set_child(module, 8)
