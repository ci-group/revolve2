import uuid
from dataclasses import dataclass, field

from ._sensor import Sensor


@dataclass
class IMUSensor(Sensor):
    """
    An inertial measurement unit.

    Reports specific force(closely related to acceleration), angular rate(closely related to angular velocity), and orientation.
    """

    _uuid: uuid.UUID = field(init=False, default_factory=uuid.uuid1)
