from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, List

from revolve2.core.physics.actor import Actor

from .analyzer import Analyzer, AnalyzerModule
from .controller import Controller
from .serialized import Serialized


class Brain(ABC):
    @abstractmethod
    def make_controller(
        self, analyzer: Analyzer, actor: Actor, dof_ids: List[AnalyzerModule]
    ) -> Controller:
        """
        Create a controller for the provided body

        :param analyzer: Tool for analyzing the modular robot
        :param actor: Actor created from the modular robot
        :param dof_ids: Map from actor joint index to AnalyzerModule
        """
        pass

    @abstractmethod
    def serialize(self) -> Serialized:
        """
        Serialize to a dictionary containing only the data types
        Dict, List, str, int, float bool,
        which in turn will only contain these data types as well.
        """
