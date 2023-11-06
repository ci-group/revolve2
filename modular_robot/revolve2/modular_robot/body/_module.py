from __future__ import annotations

import uuid
from dataclasses import dataclass, field

from ._attachment_point import AttachmentPoint
from ._color import Color
from ._right_angles import RightAngles


class Module:
    """Base class for a module for modular robots."""

    _uuid: uuid.UUID

    _attachment_points: dict[int, _AttachmentPoint]
    _rotation: float

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

    def __init__(
        self,
        rotation: float | RightAngles,
        color: Color,
        attachment_points: dict[int, AttachmentPoint],
    ) -> None:
        """
        Initialize this object.

        :param rotation: Orientation of this model relative to its parent.
        :param color: The color of the module.
        :param attachment_points: The attachment points available on a module.
        """
        self._uuid = uuid.uuid1()

        self._attachment_points = {
            key: self._AttachmentPoint(attachment_point)
            for key, attachment_point in attachment_points.items()
        }

        self._rotation = rotation if isinstance(rotation, float) else rotation.value

        self._parent = None
        self._parent_child_index = None

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

    def set_child(self, module: Module, child_index: int) -> None:
        """
        Attach a module to a slot.

        :param module: The module to attach.
        :param child_index: The slot to attach it to.
        """
        assert (
            module._parent is None
        ), "Child module already connected to a different slot."
        module._parent = self
        module._parent_child_index = child_index
        self._attachment_points[child_index].module = module

    def neighbours(self, within_range: int) -> list[Module]:
        """
        Get the neighbours of this module with a certain range of the module tree.

        :param within_range: The range in which modules are considered a neighbour. Minimum is 1.
        :returns: The neighbouring modules.
        """
        out_neighbours: list[Module] = []

        open_nodes: list[tuple[Module, Module | None]] = [
            (self, None)
        ]  # (module, came_from)

        for _ in range(within_range):
            new_open_nodes: list[tuple[Module, Module | None]] = []
            for open_node, came_from in open_nodes:
                attached_modules = [
                    attachment_point.module
                    for attachment_point in open_node.attachment_points.values()
                    if attachment_point.module is not None
                ]
                neighbours = [
                    mod
                    for mod in attached_modules + [open_node.parent]
                    if mod is not None
                    and ((came_from is None) or (mod.uuid is not came_from.uuid))
                ]
                out_neighbours.extend(neighbours)
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

    @property
    def attachment_points(self) -> dict[int, _AttachmentPoint]:
        """
        Get all attachment points of this module.

        :return: The attachment points.
        """
        return self._attachment_points

    @dataclass
    class _AttachmentPoint:
        attachment_point_reference: AttachmentPoint
        _module: Module | None = field(default_factory=lambda: None)

        @property
        def is_free(self) -> bool:
            """
            Return if the attachment-point is free.

            :return: The boolean.
            """
            return self._module is None

        @property
        def module(self) -> Module | None:
            """
            Return the module attached.

            :return: The module
            """
            return self._module

        @module.setter
        def module(self, module: Module) -> None:
            """
            Populate attachment point with module.

            :param module: The module.
            :raises Exception: If the AttachmentPoint is already occupied.
            """
            if not self.is_free:
                raise Exception("AttachmentPoint already populated.")
            self._module = module
