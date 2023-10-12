import math
from dataclasses import dataclass

import numpy as np
from pyrr import Quaternion, Vector3

from .._module import Module
from ..base._body import Body
from ._active_hinge_v2 import ActiveHingeV2
from ._brick_v2 import BrickV2
from ._core_v2 import CoreV2


class BodyV2(Body):
    """Body of a V1 modular robot."""

    def __init__(self, attachment_positions: list[int] = [0] * 4) -> None:
        """
        Initialize the Body.

        :param attachment_positions: The attachment positions for child modules on Core.
        """
        self.core = CoreV2(0.0, attachment_positions)
        super().__init__()

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

        if isinstance(module, CoreV2):
            for child_index, angle in [
                (module.FRONT, 0.0),
                (module.BACK, math.pi),
                (module.LEFT, math.pi / 2.0),
                (module.RIGHT, math.pi / 2.0 * 3),
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
        elif isinstance(module, BrickV2):
            for child_index, angle in [
                (module.FRONT, 0.0),
                (module.LEFT, math.pi / 2.0),
                (module.RIGHT, math.pi / 2.0 * 3),
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
        elif isinstance(module, ActiveHingeV2):
            child = module.children[module.ATTACHMENT]

            if child is not None:
                assert np.isclose(child.rotation % (math.pi / 2.0), 0.0)

                rotation = Quaternion.from_eulers([child.rotation, 0.0, 0.0])

                self._make_grid_recur(
                    child, position + rotation * Vector3([1.0, 0.0, 0.0]), rotation
                )
        else:
            raise NotImplementedError()
