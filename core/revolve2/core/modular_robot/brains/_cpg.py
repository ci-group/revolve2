"""
CPG brain.
Active hinges are connected if they are within 2 jumps in the modular robot tree structure.
That means, NOT grid coordinates, but tree distance.
"""

import math
from abc import ABC, abstractmethod
from typing import Dict, List, Tuple

import numpy as np
import numpy.typing as npt
from revolve2.actor_controllers.cpg import Cpg as ControllerCpg

from revolve2.actor_controller import ActorController
from revolve2.core.modular_robot import ActiveHinge, Body, Brain


class Cpg(Brain, ABC):
    def make_controller(self, body: Body, dof_ids: List[int]) -> ActorController:
        # get active hinges and sort them according to dof_ids
        active_hinges_unsorted = body.find_active_hinges()
        active_hinge_map = {
            active_hinge.id: active_hinge for active_hinge in active_hinges_unsorted
        }
        active_hinges = [active_hinge_map[id] for id in dof_ids]

        connections = self._find_connections(body, active_hinges)
        (internal_weights, external_weights) = self._make_weights(
            active_hinges, connections, body
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
        active_hinges: List[ActiveHinge],
        connections: List[Tuple[ActiveHinge, ActiveHinge]],
        body: Body,
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
        body: Body, active_hinges: List[ActiveHinge]
    ) -> List[Tuple[ActiveHinge, ActiveHinge]]:
        # sort by id, will be used later when ignoring existing connections
        active_hinges.sort(key=lambda mod: mod.id)

        connections: List[Tuple[ActiveHinge, ActiveHinge]] = []
        for active_hinge in active_hinges:
            neighbours_all = active_hinge.neighbours(within_range=2)
            # ignore existing connections and neighbours that are not an active hinge
            neighbours = [
                neighbour
                for neighbour in neighbours_all
                if neighbour.id > active_hinge.id and isinstance(neighbour, ActiveHinge)
            ]
            connections += zip([active_hinge] * len(neighbours), neighbours)

        return connections
