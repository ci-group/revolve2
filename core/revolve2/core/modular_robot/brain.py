from abc import ABC, abstractmethod

from revolve2.core.physics.actor import Actor

from .body import Body
from .controller import Controller
from .serialized import Serialized


class Brain(ABC):
    @abstractmethod
    def make_controller(self, modular_body: Body, actor: Actor) -> Controller:
        pass

    @abstractmethod
    def serialize(self) -> Serialized:
        """
        Serialize to a dictionary containing only the data types
        Dict, List, str, int, float bool,
        which in turn will only contain these data types as well.
        """
