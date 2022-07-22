from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

from revolve2.serialization import Serializable


class ActorController(Serializable, ABC):
    """Interface for actor controllers."""

    @abstractmethod
    def step(self, dt: float) -> None:
        """Step forward the controller dt seconds."""
        pass

    @abstractmethod
    def get_dof_targets(self) -> List[float]:
        """Get the degree of freedom targets from the controller."""
        pass
