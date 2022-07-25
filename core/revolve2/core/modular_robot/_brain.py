from abc import ABC, abstractmethod
from typing import List

from revolve2.actor_controller import ActorController

from ._body import Body


class Brain(ABC):
    @abstractmethod
    def make_controller(self, body: Body, dof_ids: List[int]) -> ActorController:
        """
        Create a controller for the provided body.

        :body: The body to make the brain for.
        :dof_ids: Map from actor joint index to module id.
        :returns: The created controller.
        """
