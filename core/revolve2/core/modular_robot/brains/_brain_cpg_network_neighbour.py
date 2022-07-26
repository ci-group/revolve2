import math
from abc import ABC, abstractmethod
from typing import List, Tuple

from revolve2.actor_controller import ActorController
from revolve2.actor_controllers.cpg import CpgActorController as ControllerCpg
from revolve2.core.modular_robot import ActiveHinge, Body, Brain

from ._make_cpg_network_structure_neighbour import make_cpg_network_structure_neighbour


class BrainCpgNetworkNeighbour(Brain, ABC):
    """
    A CPG brain with active hinges that are connected if they are within 2 jumps in the modular robot tree structure.

    That means, NOT grid coordinates, but tree distance.
    """

    def make_controller(self, body: Body, dof_ids: List[int]) -> ActorController:
        """
        Create a controller for the provided body.

        :body: The body to make the brain for.
        :dof_ids: Map from actor joint index to module id.
        :return: The created controller.
        """
        # get active hinges and sort them according to dof_ids
        active_hinges_unsorted = body.find_active_hinges()
        active_hinge_map = {
            active_hinge.id: active_hinge for active_hinge in active_hinges_unsorted
        }
        active_hinges = [active_hinge_map[id] for id in dof_ids]

        cpg_network_structure = make_cpg_network_structure_neighbour(active_hinges)
        connections = [
            (
                active_hinges[pair.cpg_index_lowest.index],
                active_hinges[pair.cpg_index_highest.index],
            )
            for pair in cpg_network_structure.connections
        ]

        (internal_weights, external_weights) = self._make_weights(
            active_hinges, connections, body
        )
        weight_matrix = cpg_network_structure.make_weight_matrix(
            {
                cpg: weight
                for cpg, weight in zip(cpg_network_structure.cpgs, internal_weights)
            },
            {
                pair: weight
                for pair, weight in zip(
                    cpg_network_structure.connections, external_weights
                )
            },
        )
        initial_state = cpg_network_structure.make_uniform_state(0.5 * math.sqrt(2))
        dof_ranges = cpg_network_structure.make_uniform_dof_ranges(1.0)

        return ControllerCpg(
            initial_state, cpg_network_structure.num_cpgs, weight_matrix, dof_ranges
        )

    @abstractmethod
    def _make_weights(
        self,
        active_hinges: List[ActiveHinge],
        connections: List[Tuple[ActiveHinge, ActiveHinge]],
        body: Body,
    ) -> Tuple[List[float], List[float]]:
        """
        Define the weights between neurons.

        :param active_hinges: The active hinges corresponding to each cpg.
        :param connections: Pairs of active hinges corresponding to pairs of cpgs that are connected.
                            Connection is from hinge 0 to hinge 1.
                            Opposite connection is not provided as weights are assumed to be negative.
        :param body: The body that matches this brain.
        :return: Two lists. The first list contains the internal weights in cpgs, corresponding to `active_hinges`
                 The second list contains the weights between connected cpgs, corresponding to `connections`
                 The lists should match the order of the input parameters.
        """
