import math
from dataclasses import dataclass

import numpy as np
from pyrr import Quaternion, Vector3

from ._active_hinge import ActiveHinge
from ._brick import Brick
from ._core import Core
from ._module import Module


class Body:
    """Body of a modular robot."""

    core: Core

    def __init__(self) -> None:
        """Initialize this object."""
        self.core = Core(0.0)

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

        parent = module._parent
        child_index = module._parent_child_index
        while parent is not None and child_index is not None:
            child = parent.children[child_index]
            assert child is not None
            assert np.isclose(child.rotation % (math.pi / 2.0), 0.0)

            position = Quaternion.from_eulers((child.rotation, 0.0, 0.0)) * position
            position += Vector3([1, 0, 0])
            rotation: Quaternion
            if isinstance(parent, Core):
                if child_index == Core.FRONT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, 0.0))
                elif child_index == Core.LEFT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 1))
                elif child_index == Core.BACK:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 2))
                elif child_index == Core.RIGHT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 3))
                else:
                    raise NotImplementedError()
            elif isinstance(parent, Brick):
                if child_index == Brick.FRONT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, 0.0))
                elif child_index == Brick.LEFT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 1))
                elif child_index == Brick.RIGHT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 3))
                else:
                    raise NotImplementedError()
            elif isinstance(parent, ActiveHinge):
                if child_index == ActiveHinge.ATTACHMENT:
                    rotation = Quaternion()
                else:
                    raise NotImplementedError()
            else:
                raise NotImplementedError()
            position = rotation * position
            position = Vector3.round(position)

            child_index = parent._parent_child_index
            parent = parent._parent

        return position

    def to_grid(
        self,
    ) -> tuple[list[list[list[Module | None]]], tuple[int, int, int]]:
        """
        Convert the tree structure to a grid.

        The distance between all modules is assumed to be one grid cell.
        All module angles must be multiples of 90 degrees.

        The grid is indexed depth, width, height, or x, y, z, from the perspective of the core.

        :returns: The created grid with cells set to either a Module or None and a tuple representing the position of the core.
        :raises NotImplementedError: In case a module is encountered that is not supported.

        # noqa: DAR402 NotImplementedError
        """
        return _GridMaker().make_grid(self)


class _GridMaker:
    @dataclass
    class _Cell:
        x: int
        y: int
        z: int
        module: Module

    _core_pos: tuple[int, int, int]
    _cells: list[_Cell]

    def __init__(self) -> None:
        self._cells = []

    def make_grid(
        self, body: Body
    ) -> tuple[list[list[list[Module | None]]], tuple[int, int, int]]:
        self._make_grid_recur(body.core, Vector3(), Quaternion())

        minx = min([cell.x for cell in self._cells])
        maxx = max([cell.x for cell in self._cells])
        miny = min([cell.y for cell in self._cells])
        maxy = max([cell.y for cell in self._cells])
        minz = min([cell.z for cell in self._cells])
        maxz = max([cell.z for cell in self._cells])

        depth = maxx - minx + 1
        width = maxy - miny + 1
        height = maxz - minz + 1

        grid: list[list[list[Module | None]]] = []
        for _ in range(depth):
            y: list[list[Module | None]] = []
            for _ in range(width):
                y.append([None] * (height))
            grid.append(y)

        for cell in self._cells:
            grid[cell.x - minx][cell.y - miny][cell.z - minz] = cell.module

        return grid, (-minx, -miny, -minz)

    def _make_grid_recur(
        self, module: Module, position: Vector3, orientation: Quaternion
    ) -> None:
        self._cells.append(
            self._Cell(round(position.x), round(position.y), round(position.z), module)
        )

        if isinstance(module, Core):
            for child_index, angle in [
                (Core.FRONT, 0.0),
                (Core.BACK, math.pi),
                (Core.LEFT, math.pi / 2.0),
                (Core.RIGHT, math.pi / 2.0 * 3),
            ]:
                child = module.children[child_index]

                if child is not None:
                    assert np.isclose(child.rotation % (math.pi / 2.0), 0.0)

                    rotation = (
                        orientation
                        * Quaternion.from_eulers([0.0, 0.0, angle])
                        * Quaternion.from_eulers([child.rotation, 0, 0])
                    )

                    self._make_grid_recur(
                        child, position + rotation * Vector3([1.0, 0.0, 0.0]), rotation
                    )
        elif isinstance(module, Brick):
            for child_index, angle in [
                (Brick.FRONT, 0.0),
                (Brick.LEFT, math.pi / 2.0),
                (Brick.RIGHT, math.pi / 2.0 * 3),
            ]:
                child = module.children[child_index]

                if child is not None:
                    assert np.isclose(child.rotation % (math.pi / 2.0), 0.0)

                    rotation = (
                        orientation
                        * Quaternion.from_eulers([0.0, 0.0, angle])
                        * Quaternion.from_eulers([child.rotation, 0, 0])
                    )

                    self._make_grid_recur(
                        child, position + rotation * Vector3([1.0, 0.0, 0.0]), rotation
                    )
        elif isinstance(module, ActiveHinge):
            child = module.children[ActiveHinge.ATTACHMENT]

            if child is not None:
                assert np.isclose(child.rotation % (math.pi / 2.0), 0.0)

                rotation = Quaternion.from_eulers([child.rotation, 0.0, 0.0])

                self._make_grid_recur(
                    child, position + rotation * Vector3([1.0, 0.0, 0.0]), rotation
                )
        else:
            raise NotImplementedError()


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
        for child in module.children:
            if child is not None:
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
        for child in module.children:
            if child is not None:
                self._find_recur(child)
