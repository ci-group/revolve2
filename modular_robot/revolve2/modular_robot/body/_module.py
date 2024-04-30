from __future__ import annotations

import uuid

from pyrr import Quaternion

from ._attachment_point import AttachmentPoint
from ._color import Color
from .sensors import ActiveHingeSensor, CameraSensor, IMUSensor, Sensor


class _AttachedSensors:
    """A class that contains all the attached Sensors of a Module."""

    _camera_sensor: CameraSensor | None
    _active_hinge_sensor: ActiveHingeSensor | None
    _imu_sensor: IMUSensor | None

    def __init__(self) -> None:
        """
        Initialize the AttachedSensors.

        By default, we do not have sensors on base modules.
        """
        self._camera_sensor = None
        self._imu_sensor = None
        self._active_hinge_sensor = None

    def add_sensor(self, sensor: Sensor) -> None:
        """
        Add a sensor to the attached Sensors of the module.

        :param sensor: The sensor.
        :raises KeyError: If something went wrong with attaching the sensor.
        """
        match sensor:
            case CameraSensor():
                if self._camera_sensor is None:
                    self._camera_sensor = sensor
                else:
                    raise KeyError("Camera sensor already populated")
            case IMUSensor():
                if self._imu_sensor is None:
                    self._imu_sensor = sensor
                else:
                    raise KeyError("IMU sensor already populated")
            case ActiveHingeSensor():
                if self._active_hinge_sensor is None:
                    self._active_hinge_sensor = sensor
                else:
                    raise KeyError("ActiveHinge sensor already populated")
            case _:
                raise KeyError(
                    f"Sensor of type {type(sensor)} is not defined in _module._AttachedSensors."
                )

    def get_all_sensors(self) -> list[Sensor]:
        """
        Get all sensors attached to the Module.

        Sensors that are None will not be included in the list.

        :return: The sensors.
        """
        sensors = [self._active_hinge_sensor, self._imu_sensor, self._camera_sensor]
        return [s for s in sensors if s is not None]

    @property
    def imu_sensor(self) -> IMUSensor | None:
        """
        Get the potential IMU Sensor.

        :returns: The IMU Sensor or None.
        """
        return self._imu_sensor

    @property
    def active_hinge_sensor(self) -> ActiveHingeSensor | None:
        """
        Get the potential Active Hinge Sensor.

        :returns: The ActiveHinge Sensor or None.
        """
        return self._active_hinge_sensor

    @property
    def camera_sensor(self) -> CameraSensor | None:
        """
        Get the potential Camera Sensor.

        :returns: The Camera Sensor or None.
        """
        return self._camera_sensor


class Module:
    """Base class for a module for modular robots."""

    _uuid: uuid.UUID

    _attachment_points: dict[int, AttachmentPoint]
    _children: dict[int, Module]
    _orientation: Quaternion

    _parent: Module | None
    """
    The parent module of this module.
    
    None if this module has not yet been added to a body or is the origin of the body.
    """

    _parent_child_index: int | None
    """
    Index of this module in the parent modules child list.
    
    None if this module has not yet been added to a body.
    """

    _sensors: _AttachedSensors
    _color: Color

    def __init__(
        self,
        orientation: Quaternion,
        color: Color,
        attachment_points: dict[int, AttachmentPoint],
        sensors: list[Sensor],
    ) -> None:
        """
        Initialize this object.

        :param orientation: Orientation of this model relative to its parent.
        :param color: The color of the module.
        :param attachment_points: The attachment points available on a module.
        :param sensors: The sensors associated with the module.
        """
        """Set default values."""
        self._parent = None
        self._parent_child_index = None
        self._children = {}
        self._uuid = uuid.uuid1()

        """Set parsed arguments."""
        self._sensors = _AttachedSensors()  # Initialize the attached sensors.
        for sensor in sensors:  # Add all desired sensors to the module.
            self._sensors.add_sensor(sensor)
        self._attachment_points = attachment_points
        self._orientation = orientation
        self._color = color

    @property
    def uuid(self) -> uuid.UUID:
        """
        Get the uuid.

        :returns: The uuid.
        """
        return self._uuid

    @property
    def orientation(self) -> Quaternion:
        """
        Get the orientation of this model relative to its parent.

        :returns: The orientation.
        """
        return self._orientation

    @property
    def parent(self) -> Module | None:
        """
        Get the parent module of this module.

        None if this module has not yet been added to a body or is the origin of the body.

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

    @property
    def children(self) -> dict[int, Module]:
        """
        Get all children on this module.

        :return: The children and their respective attachment point index.
        """
        return self._children

    def set_child(self, module: Module, child_index: int) -> None:
        """
        Attach a module to certain AttachmentPoint.

        :param module: The module to attach.
        :param child_index: The index of the AttachmentPoint to attach it to.
        :raises KeyError: If attachment point is already populated.
        """
        assert (
            module._parent is None
        ), "Child module already connected to a different slot."
        module._parent = self
        module._parent_child_index = child_index
        if self.can_set_child(child_index):
            self._children[child_index] = module
        else:
            raise KeyError("Attachment point already populated")

    def can_set_child(self, child_index: int) -> bool:
        """
        Check if a child can be set on a specific attachment point on the module.

        :param child_index: The child index.
        :return: The boolean value.
        """
        if self._children.get(child_index, True):
            return True
        return False

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
                    self._children.get(index)
                    for index in open_node.attachment_points.keys()
                    if self._children.get(index) is not None
                ]
                neighbours = [
                    mod
                    for mod in attached_modules + [open_node.parent]
                    if mod is not None
                    and (came_from is None or mod.uuid is not came_from.uuid)
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

    @color.setter
    def color(self, color: Color) -> None:
        """
        Set the color of a module.

        :param color: The color
        """
        self._color = color

    @property
    def attachment_points(self) -> dict[int, AttachmentPoint]:
        """
        Get all attachment points of this module.

        :return: The attachment points.
        """
        return self._attachment_points

    @property
    def sensors(self) -> _AttachedSensors:
        """
        Get the sensors.

        :return: The value.
        """
        return self._sensors

    def add_sensor(self, sensor: Sensor) -> None:
        """
        Add a sensor to the module.

        :param sensor: The sensor.
        """
        self._sensors.add_sensor(sensor)
