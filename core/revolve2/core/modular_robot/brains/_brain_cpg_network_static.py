from typing import List

import numpy as np
import numpy.typing as npt
from revolve2.actor_controller import ActorController
from revolve2.actor_controllers.cpg import CpgActorController as ControllerCpg
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
