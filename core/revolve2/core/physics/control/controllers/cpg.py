from typing import List

import numpy as np
from revolve2.core.physics.control import Controller
from scipy.integrate import solve_ivp


class Cpg(Controller):
    _weight_matrix: np.ndarray  # nxn matrix matching number of neurons
    _num_output_neurons: int
    _state: np.ndarray

    def __init__(
        self,
        initial_state: np.ndarray,
        num_output_neurons: int,
        weight_matrix: np.ndarray,
    ):
        """
        First num_output_neurons will be dof targets
        """
        assert initial_state.ndim == 1
        assert weight_matrix.ndim == 2
        assert weight_matrix.shape[0] == weight_matrix.shape[1]
        assert initial_state.shape[0] == weight_matrix.shape[0]

        self._weight_matrix = weight_matrix
        self._num_output_neurons = num_output_neurons
        self._state = initial_state

    def step(self, dt: float) -> None:
        self._state = solve_ivp(
            lambda _, state: np.matmul(self._weight_matrix, state),
            (0.0, dt),
            self._state,
            method="RK45",
        ).y[:, -1]

    def get_dof_targets(self) -> List[float]:
        return list(self._state[0 : self._num_output_neurons])
