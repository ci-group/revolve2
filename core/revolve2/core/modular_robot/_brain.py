from abc import ABC, abstractmethod
from typing import List

from revolve2.core.physics.actor import Actor
from revolve2.object_controller import ObjectController

from ._analyzer import Analyzer
from ._analyzer_module import AnalyzerModule


class Brain(ABC):
    @abstractmethod
    def make_controller(
        self, analyzer: Analyzer, actor: Actor, dof_ids: List[AnalyzerModule]
    ) -> ObjectController:
        """
        Create a controller for the provided body

        :param analyzer: Tool for analyzing the modular robot
        :param actor: Actor created from the modular robot
        :param dof_ids: Map from actor joint index to AnalyzerModule
        """
        pass
