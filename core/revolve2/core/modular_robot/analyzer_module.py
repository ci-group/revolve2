from __future__ import annotations

import math
from typing import List, Optional

import numpy as np
from pyrr import Quaternion, Vector3

from .active_hinge import ActiveHinge
from .brick import Brick
from .core import Core
from .module import Module


class AnalyzerModule:
    _id: int
    _module: Module
    _children: List[Optional[AnalyzerModule]]
    _parent: Optional[AnalyzerModule]
    _parent_child_index: Optional[int]

    def __init__(
        self,
        module: Module,
        id: int,
        parent: Optional[AnalyzerModule],
        parent_child_index: Optional[int],
    ):
        self._module = module
        self._id = id
        self._children = [None] * module.num_children

        assert (parent is None and parent_child_index is None) or (
            parent is not None and parent_child_index is not None
        )
        self._parent = parent
        self._parent_child_index = parent_child_index

    def get_child(self, index: int) -> Optional[AnalyzerModule]:
        return self._children[index]

    def set_child(self, index: int, module: AnalyzerModule) -> None:
        self._children[index] = module

    def grid_position(self) -> Vector3:
        """
        Calculate the position of this module in a 3d grid with the core as center.
        The distance between all modules is assumed to be one grid cell.
        All slot angles must be multiples of 90 degrees.
        """
        position = Vector3()

        parent = self._parent
        child_index = self._parent_child_index
        while parent is not None and child_index is not None:
            slot = parent.module.get_child(child_index)
            assert slot is not None
            assert np.isclose(slot.rotation % (math.pi / 2.0), 0.0)

            position = Quaternion.from_eulers((slot.rotation, 0.0, 0.0)) * position
            position += Vector3([1, 0, 0])
            rotation: Quaternion
            if parent.type == Module.Type.CORE:
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
            elif parent.type == Module.Type.BRICK:
                if child_index == Brick.FRONT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, 0.0))
                elif child_index == Brick.LEFT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 1))
                elif child_index == Brick.BACK:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 2))
                elif child_index == Brick.RIGHT:
                    rotation = Quaternion.from_eulers((0.0, 0.0, math.pi / 2.0 * 3))
                else:
                    raise NotImplementedError()
            elif parent.type == Module.Type.ACTIVE_HINGE:
                if child_index == ActiveHinge.ATTACHMENT_INDEX:
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

    @property
    def num_children(self) -> int:
        return len(self._children)

    @property
    def parent(self) -> Optional[AnalyzerModule]:
        return self._parent

    @property
    def type(self) -> Module.Type:
        return self.module.type

    @property
    def id(self) -> int:
        return self._id

    @property
    def module(self) -> Module:
        return self._module
