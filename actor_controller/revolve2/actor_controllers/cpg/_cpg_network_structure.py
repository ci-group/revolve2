from dataclasses import dataclass
from typing import Dict, List, Set

import numpy as np
import numpy.typing as npt


@dataclass(frozen=True)
class CpgIndex:
    index: int


@dataclass(frozen=True, init=False)
class CpgPair:
    # lowest is automatically set to be the lowest state index of the two
    cpg_index_lowest: CpgIndex
    cpg_index_highest: CpgIndex

    def __init__(self, cpg_1: CpgIndex, cpg_2: CpgIndex) -> None:
        # hacky but normal variable setting not possible with frozen enabled
        # https://stackoverflow.com/questions/57893902/how-can-i-set-an-attribute-in-a-frozen-dataclass-custom-init-method
        if cpg_1.index < cpg_2.index:
            object.__setattr__(self, "cpg_index_lowest", cpg_1)
            object.__setattr__(self, "cpg_index_highest", cpg_2)
        else:
            object.__setattr__(self, "cpg_index_lowest", cpg_2)
            object.__setattr__(self, "cpg_index_highest", cpg_1)


class CpgNetworkStructure:
    cpgs: List[CpgIndex]
    connections: Set[CpgPair]

    def __init__(self, cpgs: List[CpgIndex], connections: Set[CpgPair]) -> None:
        assert isinstance(connections, set)

        self.cpgs = cpgs
        self.connections = connections

    @staticmethod
    def make_cpgs(num_cpgs: int) -> List[CpgIndex]:
        return [CpgIndex(index) for index in range(num_cpgs)]

    def make_weight_matrix(
        self,
        internal_weights: Dict[CpgIndex, float],
        external_weights: Dict[CpgPair, float],
    ) -> npt.NDArray[np.float_]:
        state_size = self.num_cpgs * 2

        assert set(internal_weights.keys()) == set(self.cpgs)
        assert set(external_weights.keys()) == self.connections

        weight_matrix = np.zeros((state_size, state_size))

        for cpg, weight in internal_weights.items():
            weight_matrix[cpg.index][self.num_cpgs + cpg.index] = weight
            weight_matrix[self.num_cpgs + cpg.index][cpg.index] = -weight

        for cpg_pair, weight in external_weights.items():
            weight_matrix[cpg_pair.cpg_index_lowest.index][
                cpg_pair.cpg_index_highest.index
            ] = weight
            weight_matrix[cpg_pair.cpg_index_highest.index][
                cpg_pair.cpg_index_lowest.index
            ] = -weight

        return weight_matrix

    @property
    def num_params(self) -> int:
        return len(self.cpgs) + len(self.connections)

    def make_weight_matrix_from_params(
        self, params: List[float]
    ) -> npt.NDArray[np.float_]:
        assert len(params) == self.num_params

        internal_weights = {cpg: weight for cpg, weight in zip(self.cpgs, params)}

        external_weights = {
            pair: weight for pair, weight in zip(self.connections, params)
        }

        return self.make_weight_matrix(internal_weights, external_weights)

    @property
    def num_states(self) -> int:
        return len(self.cpgs) * 2

    def make_uniform_state(self, value: float) -> npt.NDArray[np.float_]:
        return np.full(self.num_states, value)

    @property
    def num_cpgs(self) -> int:
        return len(self.cpgs)

    def make_uniform_dof_ranges(self, value: float) -> npt.NDArray[np.float_]:
        return np.full(self.num_cpgs, value)
