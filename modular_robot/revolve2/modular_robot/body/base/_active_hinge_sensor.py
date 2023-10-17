import uuid
from dataclasses import dataclass, field


@dataclass
class ActiveHingeSensor:
    """A sensor for an active hinge that measures its angle."""

    _uuid: uuid.UUID = field(init=False, default_factory=uuid.uuid1)

    @property
    def uuid(self) -> uuid.UUID:
        """
        Get the uuid.

        :returns: The uuid.
        """
        return self._uuid
