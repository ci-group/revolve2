from pyrr import Quaternion, Vector3
from revolve2.actor_controller import ActorController

from ._environment_actor_controller import EnvironmentActorController
from ._terrain import Terrain
from .actor import Actor
from .running import Environment, PosedActor


def create_environment_single_actor(
    actor: Actor, controller: ActorController, terrain: Terrain
) -> Environment:
    """
    Create an environment for simulating a single actor.

    :param actor: The actor to simulate.
    :param controller: The controller for the actor.
    :param terrain: The terrain to simulate the actor in.
    :returns: The created environment.
    """
    bounding_box = actor.calc_aabb()

    env = Environment(EnvironmentActorController(controller))
    env.static_geometries.extend(terrain.static_geometry)
    env.actors.append(
        PosedActor(
            actor,
            Vector3(
                [
                    0.0,
                    0.0,
                    bounding_box.size.z / 2.0 - bounding_box.offset.z,
                ]
            ),
            Quaternion(),
            [0.0 for _ in controller.get_dof_targets()],
        )
    )
    return env
