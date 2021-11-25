from abc import ABC, abstractmethod
from typing import List, Tuple

from .batch import Batch
from .state import State


class Runner(ABC):
    @abstractmethod
    async def run_batch(self, batch: Batch) -> List[Tuple[float, State]]:
        pass
