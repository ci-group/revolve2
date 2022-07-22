from typing import List

import numpy as np
import numpy.typing as npt
from revolve2.actor_controller import ActorController
from revolve2.actor_controllers.cpg import CpgActorController as ControllerCpg
from revolve2.core.modular_robot import Body, Brain


class BrainCpgNetworkStatic(Brain):
    _initial_state: npt.NDArray[np.float_]
    _num_cpgs: int
    _weight_matrix: npt.NDArray[np.float_]
    _dof_ranges: npt.NDArray[np.float_]

    def __init__(
        self,
        initial_state: npt.NDArray[np.float_],
        num_cpgs: int,
        weight_matrix: npt.NDArray[np.float_],
        dof_ranges: npt.NDArray[np.float_],
    ) -> None:
        self._initial_state = initial_state
        self._num_cpgs = num_cpgs
        self._weight_matrix = weight_matrix
        self._dof_ranges = dof_ranges

    def make_controller(self, body: Body, dof_ids: List[int]) -> ActorController:
        return ControllerCpg(
            self._initial_state,
            self._num_cpgs,
            self._weight_matrix,
            self._dof_ranges,
        )
