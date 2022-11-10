"""Runner class."""

from abc import ABC, abstractmethod
from typing import Optional

from ._batch import Batch
from ._record_settings import RecordSettings
from ._results import BatchResults


class Runner(ABC):
    """
    Interface class for physics runners.

    Running happens either in simulation or reality, depending on the implementation.
    """

    @abstractmethod
    async def run_batch(
        self, batch: Batch, record_settings: Optional[RecordSettings] = None
    ) -> BatchResults:
        """
        Run the provided batch by simulating each contained environment.

        :param batch: The batch to run.
        :param record_settings: Optional settings for recording the runnings. If None, no recording is made.
        :returns: List of simulation states in ascending order of time.
        """
        pass
