from __future__ import annotations

from typing import List

import numpy as np
import numpy.typing as npt
from revolve2.actor_controller import ActorController
from revolve2.actor_controllers.cpg import CpgActorController as ControllerCpg
from revolve2.actor_controllers.cpg import CpgNetworkStructure
from revolve2.core.modular_robot import Body, Brain


class BrainCpgNetworkStatic(Brain):
    """
    A CPG brain with cpgs and connections defined by the user.

    A state vector is integrated over time using a weight matrix which multiplication with the state vector sum defines the derivative of the state vector.
    I.e X' = WX

    The first `num_output_neurons` in the state vector are the outputs for the controller created by this brain.
    """

    _initial_state: npt.NDArray[np.float_]
    _num_output_neurons: int
    _weight_matrix: npt.NDArray[np.float_]
    _dof_ranges: npt.NDArray[np.float_]

    def __init__(
        self,
        initial_state: npt.NDArray[np.float_],
        num_output_neurons: int,
        weight_matrix: npt.NDArray[np.float_],
        dof_ranges: npt.NDArray[np.float_],
    ) -> None:
        """
        Initialize this object.

        :param initial_state: The initial state of each neuron.
        :param num_output_neurons: The number of outputs.
        :param weight_matrix: Matrix describing the weights between the neurons.
        :param dof_ranges: Maximum range (half the complete range) of the output of degrees of freedom.
        """
        self._initial_state = initial_state
        self._num_output_neurons = num_output_neurons
        self._weight_matrix = weight_matrix
        self._dof_ranges = dof_ranges

    @classmethod
    def create_simple(
        cls,
        params: npt.NDArray[np.float_],
        cpg_network_structure: CpgNetworkStructure,
        initial_state_uniform: float,
        dof_range_uniform: float,
    ) -> BrainCpgNetworkStatic:
        """
        Create and initialize an instance of this brain using a simplified interface.

        :param params: Parameters for the weight matrix to be created.
        :param cpg_network_structure: The cpg network structure.
        :param initial_state_uniform: Initial state to use for all neurons.
        :param dof_range_uniform: Dof range to use for all neurons.
        :returns: The created brain.
        """
        initial_state = cpg_network_structure.make_uniform_state(initial_state_uniform)
        weight_matrix = (
            cpg_network_structure.make_connection_weights_matrix_from_params(
                list(params)
            )
        )
        dof_ranges = cpg_network_structure.make_uniform_dof_ranges(dof_range_uniform)
        return BrainCpgNetworkStatic(
            initial_state,
            cpg_network_structure.num_cpgs,
            weight_matrix,
            dof_ranges,
        )

    def make_controller(self, body: Body, dof_ids: List[int]) -> ActorController:
        """
        Create a controller from this brain.

        :param body: Body is provided by the parent class but is not used in this implementation.
        :param dof_ids: Dof_ids is provided by the parent class but is not used in this implementation.
        :returns: The created controller.
        """
        return ControllerCpg(
            self._initial_state,
            self._num_output_neurons,
            self._weight_matrix,
            self._dof_ranges,
        )
