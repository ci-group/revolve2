import math
from typing import Generic, Type, TypeVar

import numpy as np
from numpy.typing import NDArray
from pyrr import Quaternion, Vector3

from .._module import Module
from ._core import Core

TModule = TypeVar("TModule", bound=Module)
TModuleNP = TypeVar("TModuleNP", bound=np.generic)


class Body:
    """Body of a modular robot."""

    _core: Core

    def __init__(self, core: Core) -> None:
        """
        Initialize this object.

        :param core: The core of the body.
        """
        self._core = core

    @classmethod
    def grid_position(cls, module: Module) -> Vector3:
        """
        Calculate the position of this module in a 3d grid with the core as center.

        The distance between all modules is assumed to be one grid cell.
        All module angles must be multiples of 90 degrees.

        :param module: The module to calculate the position for.
        :returns: The calculated position.
        :raises KeyError: In case an attachment point is not found.
        """
        position = Vector3()

        parent = module.parent
        child_index = module.parent_child_index
        while parent is not None and child_index is not None:
            child = parent.children.get(child_index)
            assert child is not None
            assert np.isclose(child.rotation % (math.pi / 2.0), 0.0)

            position = Quaternion.from_eulers((child.rotation, 0.0, 0.0)) * position
            position += Vector3([1, 0, 0])

            attachment_point = parent.attachment_points.get(child_index)

            if attachment_point is None:
                raise KeyError("No attachment point found at the specified location.")
            position = attachment_point.orientation * position
            position = Vector3.round(position)

            child_index = parent.parent_child_index
            parent = parent.parent
        return position

    @classmethod
    def __find_recur(cls, module: Module, module_type: Type[TModule]) -> list[TModule]:
        modules = []
        if isinstance(module, module_type):
            modules.append(module)
        for child in module.children.values():
            modules.extend(cls.__find_recur(child, module_type))
        return modules

    def find_modules_of_type(self, module_type: Type[TModule]) -> list[TModule]:
        """
        Find all Modules of a certain type in the robot.

        :param module_type: The type.
        :return: The list of Modules.
        """
        return self.__find_recur(self._core, module_type)

    def to_grid(self) -> tuple[NDArray[TModuleNP], Vector3[np.int_]]:
        """
        Convert the tree structure to a grid.

        The distance between all modules is assumed to be one grid cell.
        All module angles must be multiples of 90 degrees.

        The grid is indexed depth, width, height, or x, y, z, from the perspective of the core.

        :returns: The created grid with cells set to either a Module or None and a position vector of the core. The position Vector3 is dtype: int.
        """
        return _GridMaker().make_grid(self)

    @property
    def core(self) -> Core:
        """
        Get the core of the Body.

        :return: The core.
        """
        return self._core


class _GridMaker(Generic[TModuleNP]):
    _x: list[int] = []
    _y: list[int] = []
    _z: list[int] = []
    _modules: list[Module] = []

    def make_grid(self, body: Body) -> tuple[NDArray[TModuleNP], Vector3[np.int_]]:

        self._make_grid_recur(body._core, Vector3(), Quaternion())

        minx, maxx = min(self._x), max(self._x)
        miny, maxy = min(self._y), max(self._y)
        minz, maxz = min(self._z), max(self._z)

        depth = maxx - minx + 1
        width = maxy - miny + 1
        height = maxz - minz + 1

        grid = np.empty(shape=(depth, width, height), dtype=Module)
        grid.fill(None)
        for x, y, z, module in zip(self._x, self._y, self._z, self._modules):
            grid[x - minx, y - miny, z - minz] = module

        return grid, Vector3([-minx, -miny, -minz])

    def _make_grid_recur(
        self, module: Module, position: Vector3, orientation: Quaternion
    ) -> None:
        self._add(position, module)

        for child_index, attachment_point in module.attachment_points.items():
            child = module.children.get(child_index)
            if child is not None:
                assert np.isclose(child.rotation % (math.pi / 2.0), 0.0)
                rotation = (
                    orientation
                    * attachment_point.orientation
                    * Quaternion.from_eulers([child.rotation, 0, 0])
                )
                self._make_grid_recur(
                    child, position + rotation * Vector3([1.0, 0.0, 0.0]), rotation
                )

    def _add(self, position: Vector3, module: Module) -> None:
        self._modules.append(module)
        x, y, z = position
        self._x.append(int(round(x)))
        self._y.append(int(round(y)))
        self._z.append(int(round(z)))
