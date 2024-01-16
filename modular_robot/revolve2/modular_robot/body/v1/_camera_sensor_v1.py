import uuid
from dataclasses import field

from ..base import CameraSensor


class CameraSensorV1(CameraSensor):
    """Camera Sensor for the V1 Robot."""

    _uuid: uuid.UUID = field(init=False, default_factory=uuid.uuid1)

    @property
    def uuid(self) -> uuid.UUID:
        """
        Get the uuid.

        :returns: The uuid.
        """
        return self._uuid
