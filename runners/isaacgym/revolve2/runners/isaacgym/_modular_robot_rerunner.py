"""Rerun(watch) a modular robot in Isaac Gym."""

from typing import List, Optional, Union

from pyrr import Quaternion, Vector3
from revolve2.core.modular_robot import ModularRobot
from revolve2.core.physics.environment_actor_controller import (
    EnvironmentActorController,
)
from revolve2.core.physics.running import Batch, Environment, PosedActor, RecordSettings
from revolve2.runners.isaacgym import LocalRunner


class ModularRobotRerunner:
    """Rerunner for one or more robots that uses Isaac Gym."""

    async def rerun(
        self,
        robots: Union[ModularRobot, List[ModularRobot]],
        control_frequency: float,
        simulation_time: int = 1000000,
        record_settings: Optional[RecordSettings] = None,
    ) -> None:
        """
        Rerun a single robot.

        :param robots: One or more robots to simulate.
        :param control_frequency: Control frequency for the simulation. See `Batch` class from physics running.
        :param simulation_time: How long to rerun each robot for.
        :param record_settings: Optional settings for recording the runnings. If None, no recording is made.
        """
        if isinstance(robots, ModularRobot):
            robots = [robots]

        batch = Batch(
            simulation_time=simulation_time,
            sampling_frequency=0.0001,
            control_frequency=control_frequency,
        )

        for robot in robots:
            actor, controller = robot.make_actor_and_controller()
            bounding_box = actor.calc_aabb()
            env = Environment(EnvironmentActorController(controller))
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

        runner = LocalRunner(real_time=True)
        await runner.run_batch(batch, record_settings=record_settings)


if __name__ == "__main__":
    print(
        "This file cannot be ran as a script. Import it and use the contained classes instead."
    )
