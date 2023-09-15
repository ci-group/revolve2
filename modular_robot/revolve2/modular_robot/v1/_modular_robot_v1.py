from revolve2.actor_controller import ActorController
from revolve2.simulation.actor import Actor
from revolve2.modular_robot._common import ModularRobot

from ._actor_builder_V1 import ActorBuilderV1


class ModularRobotV1(ModularRobot):
    """A module robot consisting of a body and brain."""
    def make_actor_and_controller(self) -> tuple[Actor, ActorController]:
        """
        Transform this modular robot into a physics actor and corresponding controller.

        :returns: (the actor, the controller)
        """
        actor, dof_ids = self.body.to_actor(ActorBuilderV1)
        controller = self.brain.make_controller(self.body, dof_ids)
        return (actor, controller)
