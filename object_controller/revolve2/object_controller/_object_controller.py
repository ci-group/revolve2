from __future__ import annotations
from abc import ABC, abstractmethod
from typing import List


class ObjectController(ABC):
    @classmethod
    @abstractmethod
    def from_config(cls, config: str) -> ObjectController:
        pass

    @abstractmethod
    def step(self, dt: float) -> None:
        pass

    @abstractmethod
    def get_dof_targets(self) -> List[float]:
        pass

    @abstractmethod
    def export_standalone(self, output_path: str) -> None:
        """
        Export this brain as a standalone package including a requirements.txt file.
        """
        pass
