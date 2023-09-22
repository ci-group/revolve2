from __future__ import annotations

from ._not_finalized_error import NotFinalizedError
from ._right_angles import RightAngles


class Module:
    """Base class for a module for modular robots."""

    _children: list[Module | None]
    _rotation: float

    # The following members are initialized by the ModularRobot finalize function:
    _id: int | None
    _parent: Module | None
    _parent_child_index: int | None
    _attachment_position: int

    def __init__(
        self,
        num_children: int,
        rotation: float | RightAngles,
        attachment_position: int,
    ):
        """
        Initialize this object.

        :param num_children: The number of Children.
        :param rotation: The Modules rotation.
        :param attachment_position: The Modules attachment_position.
        """
        self._id = None
        self._parent = None
        self._parent_child_index = None

        self._rotation = (
            rotation.value if isinstance(rotation, RightAngles) else rotation
        )
        self._children = [None] * num_children
        self._attachment_position = attachment_position

    @property
    def children(self) -> list[Module | None]:
        """
        Get the children of this module.

        :returns: The list of children.
        """
        return self._children

    @property
    def rotation(self) -> float:
        """
        Get the orientation of this model relative to its parent.

        :returns: The orientation.
        """
        return self._rotation

    @property
    def attachment_position(self) -> int:
        """
        Get the attachment position of this module if V2.

        :returns: This module's attachment position.
        """
        return self._attachment_position

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

        open_nodes: list[tuple[Module, Module | None]] = [
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
