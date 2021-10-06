from __future__ import annotations

from typing import Dict

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
