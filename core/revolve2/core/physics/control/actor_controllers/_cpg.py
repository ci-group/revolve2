from typing import List

import numpy as np
import numpy.typing as npt

from revolve2.core.physics.control import ActorController
import os
import shutil
from pathlib import Path
import json


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
        A1: npt.NDArray[np.float_] = np.matmul(
            A, state
        )  # TODO matmul doesn't seem to be properly typed.
        A2: npt.NDArray[np.float_] = np.matmul(A, (state + dt / 2 * A1))
        A3: npt.NDArray[np.float_] = np.matmul(A, (state + dt / 2 * A2))
        A4: npt.NDArray[np.float_] = np.matmul(A, (state + dt * A3))
        return state + dt / 6 * (A1 + 2 * (A2 + A3) + A4)

    def get_dof_targets(self) -> List[float]:
        return list(self._state[0 : self._num_output_neurons])

    def export_standalone(self, output_path: str) -> None:
        if os.path.exists(output_path):
            raise RuntimeError("Output path already exists.")

        os.mkdir(output_path)
        shutil.copyfile(__file__, os.path.join(output_path, "_cpg.py"))
        shutil.copyfile(
            os.path.join(Path(__file__).parent, "_cpg_export.py"),
            os.path.join(output_path, "brain.py"),
        )

        with open(os.path.join(output_path, "settings.json"), "w") as settings:
            settings.write(
                json.dumps(
                    {
                        "weight_matrix": self._weight_matrix.tolist(),
                        "num_output_neurons": self._num_output_neurons,
                        "initial_state": self._state.tolist(),
                    },
                    indent=4,
                    sort_keys=True,
                )
            )
