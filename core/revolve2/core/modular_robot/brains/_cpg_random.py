import random
from random import Random
from typing import List, Tuple

from revolve2.core.modular_robot import AnalyzerModule

from ._cpg import Cpg


class CpgRandom(Cpg):
    _rng: Random

    def __init__(self, rng: Random) -> None:
        self._rng = rng

    def _make_weights(
        self,
        active_hinges: List[AnalyzerModule],
        connections: List[Tuple[AnalyzerModule, AnalyzerModule]],
    ) -> Tuple[List[float], List[float]]:
        # TODO use provided rng object, instead of global
        return (
            [self._rng.random() * 2.0 - 1 for _ in range(len(active_hinges))],
            [self._rng.random() * 2.0 - 1 for _ in range(len(connections))],
        )
