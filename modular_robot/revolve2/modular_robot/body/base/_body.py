import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Generic, Type, TypeVar

import numpy as np
from pyrr import Quaternion, Vector3

from .._module import Module
from ._active_hinge import ActiveHinge
from ._brick import Brick
from ._core import Core

M = TypeVar("M", bound=Module)


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

            position = Quaternion.from_eulers((child.rotation, 0.0, 0.0)) * position
            position += Vector3([1, 0, 0])
            rotation: Quaternion
            if isinstance(parent, Core):
                if child_index == parent.FRONT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, 0.0))
                elif child_index == parent.LEFT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 1))
                elif child_index == parent.BACK:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 2))
                elif child_index == parent.RIGHT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 3))
                else:
                    raise NotImplementedError()
            elif isinstance(parent, Brick):
                if child_index == parent.FRONT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, 0.0))
                elif child_index == parent.LEFT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 1))
                elif child_index == parent.RIGHT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 3))
                else:
                    raise NotImplementedError()
            elif isinstance(parent, ActiveHinge):
                if child_index == parent.ATTACHMENT:
                    rotation = Quaternion()
                else:
                    raise NotImplementedError()
            else:
                raise NotImplementedError()
            position = rotation * position
            position = Vector3.round(position)

            child_index = parent.parent_child_index
            parent = parent.parent
        return position

    def find_modules_of_type(self, module_type: Type[M]) -> list[M]:
        """
        Find all Modules of a certain type in the robot.

        :param module_type: The type.
        :return: The list of Modules.
        """

        def __find_recur(module: Module) -> None:
            if isinstance(module, module_type):
                _modules.append(module)
            for child in module.children:
                if child is not None:
                    __find_recur(child)

        _modules: list[M] = field(default_factory=lambda: [])
        __find_recur(self.core)
        return _modules

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


@dataclass
class _ModuleFinder(Generic[M]):
    _type: Type[M]
    _modules: list[M] = field(default_factory=lambda: [])

    def __init__(self, module_type: Type[M]):
        self._type = module_type

    def find(self, body: Body) -> list[M]:
        self._find_recur(body.core)
        return self._modules

    def _find_recur(self, module: Module) -> None:
        if isinstance(module, self._type):
            self._modules.append(module)
        for child in module.children:
            if child is not None:
                self._find_recur(child)
