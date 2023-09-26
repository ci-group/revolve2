from abc import ABC, abstractmethod

from revolve2.actor_controller import ActorController
from revolve2.modular_robot._body import Body


class Brain(ABC):
    """Interface for the brain of a modular robot."""

    @abstractmethod
    def make_controller(self, body: Body, dof_ids: list[int]) -> ActorController:
        """
        Create a controller for the provided body.

        :param body: The body to make the brain for.
        :param dof_ids: Map from actor joint index to module id.
        :returns: The created controller.
        """
