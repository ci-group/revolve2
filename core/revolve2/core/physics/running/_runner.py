"""
Runner class
"""

from abc import ABC, abstractmethod

from ._batch import Batch
from ._results import BatchResults


class Runner(ABC):
    """
    Interface class for physics runners.
    Running happens either in simulation or reality, depending on the implementation.
    """

    @abstractmethod
    async def run_batch(self, batch: Batch) -> BatchResults:
        """
        Simulate the provided batch.

        :return: List of simulation states in ascending order of time.
        """
        pass
