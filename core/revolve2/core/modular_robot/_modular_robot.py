from typing import Dict, Tuple

from revolve2.core.physics.actor import Actor
from revolve2.object_controller import ObjectController

from ._body import Body
from ._brain import Brain


class ModularRobot:
    body: Body
    brain: Brain

    def __init__(self, body: Body, brain: Brain):
        self.body = body
        self.brain = brain

    def make_actor_and_controller(self) -> Tuple[Actor, ObjectController]:
        from ._analyzer import Analyzer
        from ._to_actor import to_actor

        analyzer = Analyzer(self.body)
        actor, dof_ids = to_actor(analyzer)
        controller = self.brain.make_controller(analyzer, actor, dof_ids)
        return (actor, controller)
