from typing import List, cast

import numpy as np
import numpy.typing as npt

from ..actor_controller import ActorController


class Cpg(ActorController):
    _weight_matrix: npt.NDArray[np.float_]  # nxn matrix matching number of neurons
    _num_output_neurons: int
    _state: npt.NDArray[np.float_]

    def __init__(
        self,
        initial_state: npt.NDArray[np.float_],
        num_output_neurons: int,
        weight_matrix: npt.NDArray[np.float_],
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
        self._state = self._rk45(self._state, self._weight_matrix, dt)

    @staticmethod
    def _rk45(
        state: npt.NDArray[np.float_], A: npt.NDArray[np.float_], dt: float
    ) -> npt.NDArray[np.float_]:
        # TODO The scipy implementation of this function is very slow for some reason.
        # investigate the performance and accuracy differences
        A1 = np.matmul(A, state)
        A2 = np.matmul(A, (state + dt / 2 * A1))
        A3 = np.matmul(A, (state + dt / 2 * A2))
        A4 = np.matmul(A, (state + dt * A3))
        return cast(
            npt.NDArray[np.float_], state + dt / 6 * (A1 + 2 * (A2 + A3) + A4)
        )  # TODO

    def get_dof_targets(self) -> List[float]:
        return list(self._state[0 : self._num_output_neurons])
