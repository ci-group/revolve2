"""Rerun(watch) a modular robot in Mujoco."""

from pyrr import Quaternion, Vector3
from revolve2.actor_controller import ActorController
from revolve2.core.modular_robot import ModularRobot
from revolve2.core.physics.running import ActorControl, Batch, Environment, PosedActor
from revolve2.runners.mujoco import LocalRunner


class ModularRobotRerunner:
    """Rerunner for a single robot that uses mujoco."""

    _controller: ActorController

    async def rerun(self, robot: ModularRobot, control_frequency: float) -> None:
        """Rerun a single robot."""
        batch = Batch(
            simulation_time=1000000,
            sampling_frequency=0.0001,
            control_frequency=control_frequency,
            control=self._control,
        )

        actor, self._controller = robot.make_actor_and_controller()

        env = Environment()
        env.actors.append(
            PosedActor(
                actor,
                Vector3([0.0, 0.0, 0.1]),
                Quaternion(),
                [0.0 for _ in self._controller.get_dof_targets()],
            )
        )
        batch.environments.append(env)

        runner = LocalRunner(headless=False)
        await runner.run_batch(batch)

    def _control(
        self, environment_index: int, dt: float, control: ActorControl
    ) -> None:
        self._controller.step(dt)
        control.set_dof_targets(0, self._controller.get_dof_targets())


if __name__ == "__main__":
    print(
        "This file cannot be ran as a script. Import it and use the contained classes instead."
    )
