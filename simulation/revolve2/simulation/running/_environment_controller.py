from abc import ABC

from ._actor_control import ActorControl


class EnvironmentController(ABC):
    """Interface for control in physics running."""

    def control(self, dt: float, actor_control: ActorControl) -> None:
        """
        Control the environment.

        :param dt: Time since last call to this function.
        :param actor_control: Object used to interface with the environment.
        """
