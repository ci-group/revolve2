import uuid
from abc import ABC
from dataclasses import dataclass, field


@dataclass
class Sensor(ABC):
    """
    An inertial measurement unit.

    Reports specific force(closely related to acceleration), angular rate(closely related to angularvelocity), and orientation.
    """

    _uuid: uuid.UUID = field(init=False, default_factory=uuid.uuid1)

    @property
    def uuid(self) -> uuid.UUID:
        """
        Get the uuid.

        :returns: The uuid.
        """
        return self._uuid
