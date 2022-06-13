"""Rerun(watch) a modular robot in Isaac Gym."""

from typing import List, Union

from pyrr import Quaternion, Vector3
from revolve2.actor_controller import ActorController
from revolve2.core.modular_robot import ModularRobot
from revolve2.core.physics.running import ActorControl, Batch, Environment, PosedActor
from revolve2.runners.isaacgym import LocalRunner


class ModularRobotRerunner:
    """Rerunner for one or more robots that uses Isaac Gym."""

    _controllers: List[ActorController]

    async def rerun(
        self, robots: Union[ModularRobot, List[ModularRobot]], control_frequency: float
    ) -> None:
        """
        Rerun a single robot.

        :param robots: One or more robots to simulate.
        :param control_frequency: Control frequency for the simulation. See `Batch` class from physics running.
        """
        if isinstance(robots, ModularRobot):
            robots = [robots]

        self._controllers = []

        batch = Batch(
            simulation_time=1000000,
            sampling_frequency=0.0001,
            control_frequency=control_frequency,
            control=self._control,
        )

        for robot in robots:
            actor, controller = robot.make_actor_and_controller()
            self._controllers.append(controller)

            env = Environment()
            env.actors.append(
                PosedActor(
                    actor,
                    Vector3([0.0, 0.0, 0.1]),
                    Quaternion(),
                    [0.0 for _ in controller.get_dof_targets()],
                )
            )
            batch.environments.append(env)

        runner = LocalRunner(real_time=True)
        await runner.run_batch(batch)

    def _control(
        self, environment_index: int, dt: float, control: ActorControl
    ) -> None:
        self._controllers[environment_index].step(dt)
        control.set_dof_targets(
            0, self._controllers[environment_index].get_dof_targets()
        )


if __name__ == "__main__":
    print(
        "This file cannot be ran as a script. Import it and use the contained classes instead."
    )
