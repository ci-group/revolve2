"""Visualize and run a modular robot using Isaac Gym."""

import math
from random import Random

from pyrr import Quaternion, Vector3
from revolve2.actor_controller import ActorController
from revolve2.core.modular_robot import ActiveHinge, Body, Brick, ModularRobot
from revolve2.core.modular_robot.brains import BrainCpgNetworkNeighbourRandom
from revolve2.core.physics.running import ActorControl, Batch, Environment, PosedActor
from revolve2.runners.isaacgym import LocalRunner


class Simulator:
    """
    Simulator setup.

    Simulates using Isaac Gym.
    Defines a control function that steps the controller and applies the degrees of freedom the controller provides.
    """

    _controller: ActorController

    async def simulate(self, robot: ModularRobot, control_frequency: float) -> None:
        """
        Simulate a robot.

        :param robot: The robot to simulate.
        :param control_frequency: Control frequency for the simulator.
        """
        batch = Batch(
            simulation_time=1000000,
            sampling_frequency=0.0001,
            control_frequency=control_frequency,
            control=self._control,
        )

        actor, self._controller = robot.make_actor_and_controller()
        bounding_box = actor.calc_aabb()

        env = Environment()
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
                [0.0 for _ in self._controller.get_dof_targets()],
            )
        )
        batch.environments.append(env)

        runner = LocalRunner()
        await runner.run_batch(batch)

    def _control(
        self, environment_index: int, dt: float, control: ActorControl
    ) -> None:
        self._controller.step(dt)
        control.set_dof_targets(0, self._controller.get_dof_targets())


async def main() -> None:
    """Run the simulation."""
    rng = Random()
    rng.seed(5)

    body = Body()
    body.core.left = ActiveHinge(math.pi / 2.0)
    body.core.left.attachment = ActiveHinge(math.pi / 2.0)
    body.core.left.attachment.attachment = Brick(0.0)
    body.core.right = ActiveHinge(math.pi / 2.0)
    body.core.right.attachment = ActiveHinge(math.pi / 2.0)
    body.core.right.attachment.attachment = Brick(0.0)
    body.finalize()

    brain = BrainCpgNetworkNeighbourRandom(rng)
    robot = ModularRobot(body, brain)

    sim = Simulator()
    await sim.simulate(robot, 60)


if __name__ == "__main__":
    import asyncio

    asyncio.run(main())
