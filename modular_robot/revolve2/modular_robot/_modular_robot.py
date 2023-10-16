import uuid

from .body.base import Body
from .brain import Brain


class ModularRobot:
    """A module robot consisting of a body and brain."""

    _uuid: uuid.UUID

    body: Body
    brain: Brain

    def __init__(self, body: Body, brain: Brain):
        """
        Initialize this object.

        :param body: The body of the modular robot.
        :param brain: The brain of the modular robot.
        """
        self._uuid = uuid.uuid1()
        self.body = body
        self.brain = brain

    @property
    def uuid(self) -> uuid.UUID:
        """
        Get the uuid.

        :returns: The uuid.
        """
        return self._uuid
