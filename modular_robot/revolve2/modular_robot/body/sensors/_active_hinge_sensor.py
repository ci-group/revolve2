import uuid
from dataclasses import dataclass, field

from ._sensor import Sensor


@dataclass
class ActiveHingeSensor(Sensor):
    """A sensors for an active hinge that measures its angle."""

    _uuid: uuid.UUID = field(init=False, default_factory=uuid.uuid1)
