from __future__ import annotations
from typing import Optional

from revolve2.simulation.actor import Color

from ._not_finalized_error import NotFinalizedError
from ._directions import Directions


class Module:
    """Base class for a module for modular robots."""

    _children: list[Optional[Module]]
    _rotation: float

    # The following members are initialized by the ModularRobot finalize function:
    _id: Optional[int]
    _parent: Optional[Module]
    _parent_child_index: Optional[int]

    _color: Color

    def __init__(self, nr_children: int, rotation: float, color: Color):
        """
        Initialize this object.

        :param num_children: The number of children this module can have.
        :param rotation: Orientation of this model relative to its parent.
        :param color: The color of the module.
        """
        if not all(
            Directions.has(x, strict=False, max_val=nr_children)
            for x in range(nr_children)
        ):
            raise ValueError("Cannot create mudule with {nr_children=}")

        self._children = [None] * nr_children
        self._rotation = rotation

        self._id = None
        self._parent = None
        self._parent_child_index = None

        self._color = color

    @property
    def children(self) -> list[Optional[Module]]:
        """
        Get the children of this module.

        :returns: The list of children.
        """
        return self._children

    def get_child(self, which: Directions) -> Optional[Module]:
        return self.children[which]

    def set_child(self, item: Module, which: Directions) -> None:
        self.children[which] = item

    def has_children(self) -> bool:
        return any(child is not None for child in self.children)

    @property
    def rotation(self) -> float:
        """
        Get the orientation of this model relative to its parent.

        :returns: The orientation.
        """
        return self._rotation

    @property
    def id(self) -> int:
        """
        Get the id of this module.

        Only valid after the modular robot body's `finalize` function has been called.

        :returns: This module's id.
        :raises NotFinalizedError: In case the robot this is part of was not yet finalized.
        """
        if self._id is None:
            raise NotFinalizedError()
        return self._id

    @id.setter
    def id(self, id: int) -> None:
        """
        Set the id of this module.

        Can only be set once.
        Don't do this manually but let the modular robot body's `finalize` function do it.

        :param id: The id of the module.
        :raises RuntimeError: In case the id was already set before.
        """
        if self._id is not None:
            raise RuntimeError("Cannot set id twice.")
        self._id = id

    def neighbours(self, within_range: int) -> list[Module]:
        """
        Get the neighbours of this module with a certain range of the module tree.

        :param within_range: The range in which modules are considered a neighbour. Minimum is 1.
        :returns: The neighbouring modules.
        :raises NotFinalizedError: In case it is detected that the modular robot body this module is part of has not been finalized.
        """
        if self._id is None:
            raise NotFinalizedError()

        out_neighbours: list[Module] = []

        open_nodes: list[tuple[Module, Optional[Module]]] = [
            (self, None)
        ]  # (module, came_from)

        for _ in range(within_range):
            new_open_nodes: list[tuple[Module, Module | None]] = []
            for open_node, came_from in open_nodes:
                neighbours = [
                    mod
                    for mod in open_node.children + [open_node._parent]
                    if mod is not None
                    and (came_from is None or mod.id is not came_from.id)
                ]
                out_neighbours += neighbours
                new_open_nodes += list(zip(neighbours, [open_node] * len(neighbours)))
            open_nodes = new_open_nodes

        return out_neighbours

    @property
    def color(self) -> Color:
        """
        Get the color of this module.

        :returns: The color.
        """
        return self._color
