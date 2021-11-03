from typing import Dict, Tuple

from revolve2.core.physics.actor import Actor
from revolve2.core.physics.control import Controller

from .body import Body
from .brain import Brain
from .serialized import Serialized


class ModularRobot:
    body: Body
    brain: Brain

    def __init__(self, body: Body, brain: Brain):
        self.body = body
        self.brain = brain

    def serialize(self) -> Dict[str, Serialized]:
        """
        Serialize to a dictionary containing only the data types
        Dict, List, str, int, float bool,
        which in turn will only contain these data types as well.
        """

        return {"body": self.body.serialize(), "brain": self.brain.serialize()}

    def make_actor_and_controller(self) -> Tuple[Actor, Controller]:
        from .analyzer import Analyzer
        from .to_actor import to_actor

        analyzer = Analyzer(self.body)
        actor, dof_ids = to_actor(analyzer)
        controller = self.brain.make_controller(analyzer, actor, dof_ids)
        return (actor, controller)
