from __future__ import annotations
from typing import List

import numpy as np
import numpy.typing as npt

from revolve2.object_controller import ObjectController

import jsonschema


class Cpg(ObjectController):
    _weight_matrix: npt.NDArray[np.float_]  # nxn matrix matching number of neurons
    _num_output_neurons: int
    _state: npt.NDArray[np.float_]

    _CONFIG_SCHEMA = {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "type": "object",
        "properties": {
            "initial_state": {
                "type": "array",
                "items": {
                    "type": "number",
                },
            },
            "num_output_neurons": {"type": "integer"},
            "weight_matrix": {
                "type": "array",
                "items": {"type": "array", "items": {"type": "number"}},
            },
        },
        "required": ["initial_state", "num_output_neurons", "weight_matrix"],
    }

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

    @classmethod
    def from_config(cls, config: str) -> Cpg:
        jsonschema.validate(config, cls._CONFIG_SCHEMA)
        return Cpg(
            np.array(config["initial_state"]),
            config["num_output_neurons"],
            np.array(config["weight_matrix"]),
        )

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
