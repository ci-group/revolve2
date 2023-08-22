"""Contains EnvironmentActorController, an environment controller for an environment with a single actor that uses a provided ActorController."""

from revolve2.actor_controller import ActorController
from revolve2.core.physics.running import ActorControl, EnvironmentController


class EnvironmentActorController(EnvironmentController):
    """An environment controller for an environment with a single actor that uses a provided ActorController."""

    actor_controller: ActorController

    def __init__(self, actor_controller: ActorController) -> None:
        """
        Initialize this object.

        :param actor_controller: The actor controller to use for the single actor in the environment.
        """
        self.actor_controller = actor_controller

    def control(self, dt: float, actor_control: ActorControl) -> None:
        """
        Control the single actor in the environment using an ActorController.

        :param dt: Time since last call to this function.
        :param actor_control: Object used to interface with the environment.
        """
        self.actor_controller.step(dt)
        actor_control.set_dof_targets(0, self.actor_controller.get_dof_targets())
