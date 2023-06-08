from __future__ import annotations

from typing import List

import numpy as np
import numpy.typing as npt
from revolve2.actor_controller import ActorController
from revolve2.serialization import SerializeError, StaticData


class CpgActorController(ActorController):
    """
    Cpg network actor controller.

    A state array that is integrated over time following the differential equation `X'=WX`.
    W is a weight matrix that is multiplied by the state array.
    The first `num_output_neurons` are the degree of freedom outputs of the controller.
    """

    _state: npt.NDArray[np.float_]
    _num_output_neurons: int
    _weight_matrix: npt.NDArray[np.float_]  # nxn matrix matching number of neurons
    _dof_ranges: npt.NDArray[np.float_]

    def __init__(
        self,
        state: npt.NDArray[np.float_],
        num_output_neurons: int,
        weight_matrix: npt.NDArray[np.float_],
        dof_ranges: npt.NDArray[np.float_],
    ) -> None:
        """
        Initialize this object.

        :param state: The initial state of the neural network.
        :param num_output_neurons: The number of output neurons. These will be the first n neurons of the state array.
        :param weight_matrix: The weight matrix used during integration.
        :param dof_ranges: Maximum range (half the complete range) of the output of degrees of freedom.
        """
        assert state.ndim == 1
        assert weight_matrix.ndim == 2
        assert weight_matrix.shape[0] == weight_matrix.shape[1]
        assert state.shape[0] == weight_matrix.shape[0]

        self._state = state
        self._num_output_neurons = num_output_neurons
        self._weight_matrix = weight_matrix
        self._dof_ranges = dof_ranges

    def step(self, dt: float) -> None:
        """
        Step the controller dt seconds forward.

        :param dt: The number of seconds to step forward.
        """
        self._state = self._rk45(self._state, self._weight_matrix, dt)

    @staticmethod
    def _rk45(
        state: npt.NDArray[np.float_], A: npt.NDArray[np.float_], dt: float
    ) -> npt.NDArray[np.float_]:
        A1: npt.NDArray[np.float_] = np.matmul(A, state)
        A2: npt.NDArray[np.float_] = np.matmul(A, (state + dt / 2 * A1))
        A3: npt.NDArray[np.float_] = np.matmul(A, (state + dt / 2 * A2))
        A4: npt.NDArray[np.float_] = np.matmul(A, (state + dt * A3))
        return state + dt / 6 * (A1 + 2 * (A2 + A3) + A4)

    def get_dof_targets(self) -> List[float]:
        """
        Get the degree of freedom targets from the controller.

        This will be the first `num_output_neurons` states from the state array.

        :returns: The dof targets.
        """
        return list(
            np.clip(
                self._state[0 : self._num_output_neurons],
                a_min=-self._dof_ranges,
                a_max=self._dof_ranges,
            )
        )

    def serialize(self) -> StaticData:
        """
        Serialize this object.

        :returns: The serialized object.
        """
        return {
            "state": self._state.tolist(),
            "num_output_neurons": self._num_output_neurons,
            "weight_matrix": self._weight_matrix.tolist(),
            "dof_ranges": self._dof_ranges.tolist(),
        }

    @classmethod
    def deserialize(cls, data: StaticData) -> CpgActorController:
        """
        Deserialize an instance of this class from `StaticData`.

        :param data: The data to deserialize from.
        :returns: The deserialized instance.
        :raises SerializeError: If this object cannot be deserialized from the given data.
        """
        if (
            not type(data) == dict
            or not "state" in data
            or not type(data["state"]) == list
            or not all(type(s) == float for s in data["state"])
            or not "num_output_neurons" in data
            or not type(data["num_output_neurons"]) is int
            or not "weight_matrix" in data
            or not type(data["weight_matrix"]) == list
            or not all(
                type(r) == list and all(type(c) == float for c in r)
                for r in data["weight_matrix"]
            )
            or not "dof_ranges" in data
            or not type(data["dof_ranges"]) == list
            or not all(type(r) == float for r in data["dof_ranges"])
        ):
            raise SerializeError()

        return CpgActorController(
            np.array(data["state"]),
            data["num_output_neurons"],
            np.array(data["weight_matrix"]),
            np.array(data["dof_ranges"]),
        )
