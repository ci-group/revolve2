"""Rerun(watch) a modular robot in Mujoco."""

from typing import List, Union

from pyrr import Quaternion, Vector3
from revolve2.actor_controller import ActorController
from revolve2.core.modular_robot import ModularRobot
from revolve2.core.physics.running import ActorControl, Batch, Environment, PosedActor
from revolve2.runners.mujoco import LocalRunner


class ModularRobotRerunner:
    """Rerunner for a single robot that uses Mujoco."""

    _controllers: List[ActorController]

    async def rerun(
        self,
        robots: Union[ModularRobot, List[ModularRobot]],
        control_frequency: float,
        simulation_time: int = 1000000,
        start_paused: bool = False,
    ) -> None:
        """
        Rerun a single robot.

        :param robots: One or more robots to simulate.
        :param control_frequency: Control frequency for the simulation. See `Batch` class from physics running.
        :param simulation_time: How long to rerun each robot for.
        :param start_paused: If True, start the simulation paused. Only possible when not in headless mode.
        """
        if isinstance(robots, ModularRobot):
            robots = [robots]

        self._controllers = []

        batch = Batch(
            simulation_time=simulation_time,
            sampling_frequency=0.0001,
            control_frequency=control_frequency,
            control=self._control,
        )

        for robot in robots:
            actor, controller = robot.make_actor_and_controller()
            bounding_box = actor.calc_aabb()
            self._controllers.append(controller)

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
                    [0.0 for _ in controller.get_dof_targets()],
                )
            )
            batch.environments.append(env)

        runner = LocalRunner(headless=False, start_paused=start_paused)
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
