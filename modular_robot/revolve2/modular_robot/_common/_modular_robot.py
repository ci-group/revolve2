from revolve2.actor_controller import ActorController
from revolve2.simulation.actor import Actor
from ._brain import Brain
from ._body import Body

from abc import ABC, abstractmethod

class ModularRobot(ABC):
    """A module robot consisting of a body and brain."""

    body: Body
    brain: Brain

    def __init__(self, body: Body, brain: Brain):
        """
        Initialize this object.

        :param body: The body of the modular robot.
        :param brain: The brain of the modular robot.
        """
        self.body = body
        self.brain = brain

    @abstractmethod
    def make_actor_and_controller(self) -> tuple[Actor, ActorController]:
        """
        Transform this modular robot into a physics actor and corresponding controller.

        :returns: (the actor, the controller)
        """
        pass
