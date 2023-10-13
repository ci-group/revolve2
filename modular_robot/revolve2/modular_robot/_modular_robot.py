from ._has_uuid import HasUUID
from .body.base import Body
from .brain import Brain


class ModularRobot(HasUUID):
    """A module robot consisting of a body and brain."""

    body: Body
    brain: Brain

    def __init__(self, body: Body, brain: Brain):
        """
        Initialize this object.

        :param body: The body of the modular robot.
        :param brain: The brain of the modular robot.
        """
        super().__init__()
        self.body = body
        self.brain = brain
