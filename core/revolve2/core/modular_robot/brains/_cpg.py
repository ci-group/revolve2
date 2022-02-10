import math
from abc import ABC, abstractmethod
from typing import Dict, List, Tuple

import numpy as np
import numpy.typing as npt
from revolve2.core.modular_robot import Analyzer, AnalyzerModule, Brain, Module
from revolve2.core.physics.actor import Actor
from revolve2.actor_controller import ActorController
from revolve2.actor_controllers.cpg import Cpg as ControllerCpg


class Cpg(Brain, ABC):
    def make_controller(
        self, analyzer: Analyzer, actor: Actor, dof_ids: List[AnalyzerModule]
    ) -> ActorController:
        active_hinges = analyzer.active_hinges
        connections = self._find_connections(analyzer)
        (internal_weights, external_weights) = self._make_weights(
            active_hinges, connections
        )
        assert len(internal_weights) == len(active_hinges)
        assert len(external_weights) == len(connections)

        id_to_index: Dict[int, int] = {}
        for i, active_hinge in enumerate(active_hinges):
            id_to_index[active_hinge.id] = i

        intial_state: npt.NDArray[np.float_] = np.array(
            [0.5 * math.sqrt(2)] * (len(active_hinges) * 2)
        )
        weight_matrix = np.zeros((len(active_hinges) * 2, len(active_hinges) * 2))

        for i in range(len(active_hinges)):
            weight_matrix[i][len(active_hinges) + i] = internal_weights[i]
            weight_matrix[len(active_hinges) + i][i] = -internal_weights[i]

        for i, (mod_a, mod_b) in enumerate(connections):
            index_a = id_to_index[mod_a.id]
            index_b = id_to_index[mod_b.id]
            weight_matrix[index_a][index_b] = external_weights[i]
            weight_matrix[index_b][index_a] = -external_weights[i]

        return ControllerCpg(intial_state, len(active_hinges), weight_matrix)

    @abstractmethod
    def _make_weights(
        self,
        active_hinges: List[AnalyzerModule],
        connections: List[Tuple[AnalyzerModule, AnalyzerModule]],
    ) -> Tuple[List[float], List[float]]:
        """
        Override to the weights between neurons.

        :param active_hinges: The active hinges corresponding to each cpg.
        :param connection: Pairs of active hinges corresponding to pairs of cpgs that are connected.
        :return: Two lists. The first list contains the internal weights in cpgs, corresponding to `active_hinges`
                 The second list contains the weights between connected cpgs, corresponding to `connections`
                 The lists should match the order of the input parameters.
        """

    @staticmethod
    def _find_connections(
        analyzer: Analyzer,
    ) -> List[Tuple[AnalyzerModule, AnalyzerModule]]:
        active_hinges = analyzer.active_hinges
        # sort by id, will be used later when ignoring existing connections
        active_hinges.sort(key=lambda mod: mod.id)

        connections: List[Tuple[AnalyzerModule, AnalyzerModule]] = []
        for active_hinge in analyzer.active_hinges:
            neighbours = analyzer.neighbours(active_hinge, 2)
            # ignore existing connections and neighbours that are not an active hinge
            neighbours = [
                neighbour
                for neighbour in neighbours
                if neighbour.id > active_hinge.id
                and neighbour.type == Module.Type.ACTIVE_HINGE
            ]
            connections += zip([active_hinge] * len(neighbours), neighbours)

        return connections
