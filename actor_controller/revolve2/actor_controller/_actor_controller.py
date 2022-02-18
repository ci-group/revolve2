from __future__ import annotations

from abc import ABC, abstractmethod
from typing import List

from revolve2.serialization import Serializable


class ActorController(Serializable, ABC):
    @abstractmethod
    def step(self, dt: float) -> None:
        pass

    @abstractmethod
    def get_dof_targets(self) -> List[float]:
        pass
