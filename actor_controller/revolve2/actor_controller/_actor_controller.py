from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

from revolve2.serialization import Serializable


class ActorController(Serializable, ABC):
    """Interface for actor controllers."""

    @abstractmethod
    def step(self, dt: float) -> None:
        """
        Step the controller dt seconds forward.

        :param df: The number of seconds to step forward.
        """
        pass

    @abstractmethod
    def get_dof_targets(self) -> List[float]:
        """
        Get the degree of freedom targets from the controller.

        :returns: The dof targets.
        """
        pass
