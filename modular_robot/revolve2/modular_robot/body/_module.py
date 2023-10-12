from __future__ import annotations

from ._color import Color
from ._right_angles import RightAngles


class Module:
    """Base class for a module for modular robots."""

    _children: list[Module | None]
    _rotation: float

    _id: int | None
    """
    Id of this module.
    
    Unique within its parent body.
    None if this module has not yet been added to a body.
    """

    _parent: Module | None
    """
    The parent module of this module.
    
    None if this module has not yet been added to a body.
    """

    _parent_child_index: int | None
    """
    Index of this module in the parent modules child list.
    
    None if this module has not yet been added to a body.
    """

    _color: Color

    def __init__(self, num_children: int, rotation: float | RightAngles, color: Color):
        """
        Initialize this object.

        :param num_children: The number of children this module can have.
        :param rotation: Orientation of this model relative to its parent.
        :param color: The color of the module.
        """
        self._children = [None] * num_children

        self._rotation = rotation if isinstance(rotation, float) else rotation.value

        self._id = None
        self._parent = None
        self._parent_child_index = None

        self._color = color

    @property
    def children(self) -> list[Module | None]:
        """
        Get the children of this module.

        Do not alter the returned list.
        It will break stuff.

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
    def id(self) -> int | None:
        """
        Get the id of this module.

        Unique within its parent body.
        None if this module has not yet been added to a body.

        :returns: The id of this module, or None if this module has not yet been added to a body.
        """
        return self._id

    @property
    def parent(self) -> Module | None:
        """
        Get the parent module of this module.

        None if this module has not yet been added to a body.

        :returns: The parent module of this module, or None if this module has not yet been added to a body.
        """
        return self._parent

    @property
    def parent_child_index(self) -> int | None:
        """
        Index of this module in the parent modules child list.

        None if this module has not yet been added to a body.

        :returns: The index of this module in the parent modules child list, or None if this module has not yet been added to a body.
        """
        return self._parent_child_index

    def _set_id_and_parent(
        self, id: int, parent: Module, parent_child_index: int
    ) -> None:
        if (
            self._id is not None
            or self._parent is not None
            or self._parent_child_index is not None
        ):
            raise RuntimeError("Id and parent are already assigned.")
        self._id = id
        self._parent = parent
        self._parent_child_index = parent_child_index

    def set_child(self, module: Module, child_index: int) -> None:
        """
        Attach a module to a slot.

        :param module: The module to attach.
        :param child_index: The slot to attach it to.
        :raises RuntimeError: If that slot is already taken by another module.
        """
        if self.children[child_index] is not None:
            raise RuntimeError("Slot already has module.")
        module._set_id_and_parent(self._get_new_module_id(), self, child_index)
        self.children[child_index] = module

    def _get_new_module_id(self) -> int:
        """
        Get a new module id, unique within the parent body.

        :returns: The unique id.
        """
        assert self._parent is not None
        return self._parent._get_new_module_id()

    def neighbours(self, within_range: int) -> list[Module]:
        """
        Get the neighbours of this module with a certain range of the module tree.

        :param within_range: The range in which modules are considered a neighbour. Minimum is 1.
        :returns: The neighbouring modules.
        :raises RuntimeError: If this module is not part of a body.
        """
        if self._id is None:
            raise RuntimeError("Module is not part of a body.")

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

    @property
    def color(self) -> Color:
        """
        Get the color of this module.

        :returns: The color.
        """
        return self._color
