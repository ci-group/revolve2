from __future__ import annotations

import uuid

from ._color import Color
import numpy as np

class Module:
    """Base class for a module for modular robots."""

    def __init__(
        self,
        rotation: float,
        color: Color,
    ) -> None:
        """
        Initialize this object.

        :param rotation: Orientation of this model relative to its parent.
        :param color: The color of the module.
        """
        self._uuid = uuid.uuid1()
        self._connected_modules = []
        self._rotation = rotation if isinstance(rotation, float) else rotation.value
        self._color = color

    @property
    def uuid(self) -> uuid.UUID:
        """
        Get the uuid.

        :returns: The uuid.
        """
        return self._uuid

    @property
    def rotation(self) -> float:
        """
        Get the orientation of this model relative to its parent.

        :returns: The orientation.
        """
        return self._rotation

    @property
    def connected_modules(self) -> list[Module]:
        """
        Get all children on this module.

        :return: The children and their respective attachment point index.
        """
        return self._connected_modules

    def attach_module(self, module: Module) -> None:
        """
        Attach a module to a slot.

        :param module: The module to attach.
        :param child_index: The slot to attach it to.
        :raises KeyError: If attachment point is already populated.
        """
        self._connected_modules.append(module)

    def neighbours(self, depth: int = 1) -> list[Module]:
        """
        Get the neighbours of this module with a certain range of the module tree.

        :param depth: The range in which modules are considered a neighbour. Minimum is 1.
        :returns: The neighbouring modules.
        """
        assert(depth > 0)
        neighbours: list[Module] = [self]
        Q = [self]
        current_depth = 0
        
        while current_depth < depth and bool(Q):
            newQ = np.array([])
            for node in Q:
                newQ = np.append(newQ, node.connected_modules)
            
            neighbours = np.append(neighbours, newQ)
            Q = newQ.copy()
            current_depth += 1

        return neighbours

    @property
    def color(self) -> Color:
        """
        Get the color of this module.

        :returns: The color.
        """
        return self._color
