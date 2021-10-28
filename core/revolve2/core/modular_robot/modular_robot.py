from typing import Dict, Tuple

from revolve2.core.physics.actor import Actor

from .body import Body
from .brain import Brain
from .controller import Controller
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
        from .to_actor import to_actor

        actor = to_actor(self)
        controller = self.brain.make_controller(self.body, actor)
        return (actor, controller)
