import math
from abc import ABC, abstractmethod

import numpy as np
from pyrr import Quaternion, Vector3

from .._module import Module
from ._active_hinge import ActiveHinge
from ._brick import Brick
from ._core import Core


class Body(ABC):
    """Body of a modular robot."""

    core: Core

    def __init__(self) -> None:
        """Initialize this object."""

    def grid_position(self, module: Module) -> Vector3:
        """
        Calculate the position of this module in a 3d grid with the core as center.

        The distance between all modules is assumed to be one grid cell.
        All module angles must be multiples of 90 degrees.

        :param module: The module to calculate the position for.
        :returns: The calculated position.
        :raises NotImplementedError: In case a module is encountered that is not supported.
        """
        position = Vector3()

        parent = module.parent
        child_index = module.parent_child_index
        while parent is not None and child_index is not None:
            child = parent.children[child_index]
            assert child is not None
            assert np.isclose(child.rotation % (math.pi / 2.0), 0.0)

            if child_index is None:
                raise NotImplementedError()
            rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * child_index))

            position = Quaternion.from_eulers((child.rotation, 0.0, 0.0)) * position
            position += Vector3([1, 0, 0])
            position = rotation * position
            position = Vector3.round(position)

            child_index = parent.parent_child_index
            parent = parent.parent
        return position

    def find_active_hinges(self) -> list[ActiveHinge]:
        """
        Find all active hinges in the body.

        :returns: A list of all active hinges in the body
        """
        return _ActiveHingeFinder().find(self)

    def find_bricks(self) -> list[Brick]:
        """
        Find all bricks in the body.

        :returns: A list of all bricks in the body
        """
        return _BrickFinder().find(self)

    @abstractmethod
    def to_grid(self) -> tuple[list[list[list[Module | None]]], tuple[int, int, int]]:
        """
        Convert the tree structure to a grid.

        The distance between all modules is assumed to be one grid cell.
        All module angles must be multiples of 90 degrees.

        The grid is indexed depth, width, height, or x, y, z, from the perspective of the core.

        :returns: The created grid with cells set to either a Module or None and a tuple representing the position of the core.
        :raises NotImplementedError: In case a module is encountered that is not supported.
        """


class _ActiveHingeFinder:
    _active_hinges: list[ActiveHinge]

    def __init__(self) -> None:
        self._active_hinges = []

    def find(self, body: Body) -> list[ActiveHinge]:
        self._find_recur(body.core)
        return self._active_hinges

    def _find_recur(self, module: Module) -> None:
        if isinstance(module, ActiveHinge):
            self._active_hinges.append(module)
        for child in module.children.values():
            self._find_recur(child)


class _BrickFinder:
    _bricks: list[Brick]

    def __init__(self) -> None:
        self._bricks = []

    def find(self, body: Body) -> list[Brick]:
        self._find_recur(body.core)
        return self._bricks

    def _find_recur(self, module: Module) -> None:
        if isinstance(module, Brick):
            self._bricks.append(module)
        for child in module.children.values():
            self._find_recur(child)
