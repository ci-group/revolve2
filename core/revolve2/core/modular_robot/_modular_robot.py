from typing import Tuple

from revolve2.actor_controller import ActorController
from revolve2.core.physics.actor import Actor

from ._body import Body
from ._brain import Brain


class ModularRobot:
    body: Body
    brain: Brain

    def __init__(self, body: Body, brain: Brain):
        self.body = body
        self.brain = brain

    def make_actor_and_controller(self) -> Tuple[Actor, ActorController]:
        actor, dof_ids = self.body.to_actor()
        controller = self.brain.make_controller(self.body, dof_ids)
        return (actor, controller)
