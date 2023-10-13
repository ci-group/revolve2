import uuid


class HasUUID:
    """Provides a UUID to the object."""

    _uuid: uuid.UUID

    def __init__(self) -> None:
        """Set the UUID of the object."""
        self._uuid = uuid.uuid1()

    @property
    def uuid(self) -> uuid.UUID:
        """
        Get the UUID.

        :returns: The UUID.
        """
        assert hasattr(self, "_uuid")
        return self._uuid
