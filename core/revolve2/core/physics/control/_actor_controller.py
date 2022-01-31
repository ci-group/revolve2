from abc import ABC, abstractmethod
from typing import List


class ActorController(ABC):
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
