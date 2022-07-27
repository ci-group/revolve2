from typing import Tuple

from revolve2.actor_controller import ActorController
from revolve2.core.physics.actor import Actor

from ._body import Body
from ._brain import Brain


class ModularRobot:
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

    def make_actor_and_controller(self) -> Tuple[Actor, ActorController]:
        """
        Transform this modular robot into a physics actor and corresponding controller.

        :returns: (the actor, the controller)
        """
        actor, dof_ids = self.body.to_actor()
        controller = self.brain.make_controller(self.body, dof_ids)
        return (actor, controller)
