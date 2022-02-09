from __future__ import annotations
from typing import List

import numpy as np
import numpy.typing as npt
from typing import Any, cast

from revolve2.object_controller import ObjectController
from revolve2.serialization import StaticData
import jsonschema


class Cpg(ObjectController):
    _SERIALIZED_SCHEMA = {
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "type": "object",
        "properties": {
            "state": {
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
        "required": ["state", "num_output_neurons", "weight_matrix"],
    }

    _state: npt.NDArray[np.float_]
    _num_output_neurons: int
    _weight_matrix: npt.NDArray[np.float_]  # nxn matrix matching number of neurons

    def __init__(
        self,
        state: npt.NDArray[np.float_],
        num_output_neurons: int,
        weight_matrix: npt.NDArray[np.float_],
    ):
        """
        First num_output_neurons will be dof targets
        """
        assert state.ndim == 1
        assert weight_matrix.ndim == 2
        assert weight_matrix.shape[0] == weight_matrix.shape[1]
        assert state.shape[0] == weight_matrix.shape[0]

        self._state = state
        self._num_output_neurons = num_output_neurons
        self._weight_matrix = weight_matrix

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

    def serialize(self) -> StaticData:
        return {
            "state": self._state.tolist(),
            "num_output_neurons": self._num_output_neurons,
            "weight_matrix": self._weight_matrix.tolist(),
        }

    @classmethod
    def deserialize(cls, data: StaticData) -> Cpg:
        jsonschema.validate(data, cls._SERIALIZED_SCHEMA)
        return Cpg(
            np.array(data["state"]),  # type: ignore # TODO mypy does not understand json schema
            data["num_output_neurons"],  # type: ignore # TODO mypy does not understand json schema
            np.array(data["weight_matrix"]),  # type: ignore # TODO mypy does not understand json schema
        )
