import uuid
from dataclasses import dataclass, field

from pyrr import Vector3

from revolve2.simulation.scene.vector2 import Vector2

from ._sensor import Sensor


@dataclass
class CameraSensor(Sensor):
    """A camera for the Modular Robot."""

    position: Vector3  # The position of the camera on the parent module.
    camera_size: Vector2 = field(
        init=False, default_factory=Vector2([200, 200])
    )  # The size of the image produced by the camera (w x h).
    _uuid: uuid.UUID = field(init=False, default_factory=uuid.uuid1)
