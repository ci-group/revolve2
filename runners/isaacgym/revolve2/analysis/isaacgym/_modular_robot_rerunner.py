"""
Rerun(watch) a modular robot in isaac gym.
"""

from pyrr import Quaternion, Vector3
from revolve2.core.modular_robot import ModularRobot
from revolve2.object_controller import ObjectController
from revolve2.core.physics.running import ActorControl, Batch, Environment, PosedActor
from revolve2.runners.isaacgym import LocalRunner


class ModularRobotRerunner:
    _controller: ObjectController

    async def rerun(self, robot: ModularRobot, control_frequency: float) -> None:
        batch = Batch(
            simulation_time=1000000,
            sampling_frequency=0.0001,
            control_frequency=control_frequency,
            control=self._control,
        )

        actor, self._controller = robot.make_actor_and_controller()

        env = Environment()
        env.actors.append(PosedActor(actor, Vector3([0.0, 0.0, 0.1]), Quaternion()))
        batch.environments.append(env)

        runner = LocalRunner(LocalRunner.SimParams())
        await runner.run_batch(batch)

    def _control(self, dt: float, control: ActorControl) -> None:
        self._controller.step(dt)
        control.set_dof_targets(0, 0, self._controller.get_dof_targets())


if __name__ == "__main__":
    print(
        "This file cannot be ran as a script. Import it and use the contained classes instead."
    )
