"""Runner class."""

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
        Run the provided batch by simulating each contained environment.

        :param batch: The batch to run.
        :return: List of simulation states in ascending order of time.
        """
        pass
