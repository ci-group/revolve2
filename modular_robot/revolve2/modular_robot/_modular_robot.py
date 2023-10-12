from .body.base import Body
from .brain import Brain


class ModularRobot:
    """A module robot consisting of a body and brain."""

    _id: int | None
    """
    Uniquely identifies the object within an arbitrary namespace.
    
    Whatever this means it not defined by this class.
    It is simply a variable that can be used to identify the object by a user using their own protocol.
    """

    body: Body
    brain: Brain

    def __init__(self, body: Body, brain: Brain):
        """
        Initialize this object.

        :param body: The body of the modular robot.
        :param brain: The brain of the modular robot.
        """
        self._id = None
        self.body = body
        self.brain = brain

    @property
    def id(self) -> int | None:
        """
        Get the id of this modular robot.

        :returns: The id, or None if it has not been set.
        """
        return self._id

    @id.setter
    def id(self, id: int) -> None:
        """
        Set the id of this modular robot.

        Do not use this if you do not know what you are doing.

        :param id: The new id of the modular robot.
        :raises RuntimeError: If the robot already has an id assigned.
        """
        if self._id is not None:
            raise RuntimeError("Id has already been assigned.")
        self._id = id
