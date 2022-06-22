import math
import tempfile
from typing import List

import mujoco
import mujoco_viewer
from dm_control import mjcf, mujoco, viewer
from pyrr import Quaternion, Vector3

from revolve2.core.physics.actor.urdf import to_urdf as physbot_to_urdf
from revolve2.core.physics.running import (
    ActorControl,
    ActorState,
    Batch,
    BatchResults,
    EnvironmentState,
    Runner,
    Environment,
)


class LocalRunner(Runner):
    _headless: bool

    def __init__(self, headless: bool = False):
        self._headless = headless

    async def run_batch(self, batch: Batch) -> BatchResults:
        control_step = 1 / batch.control_frequency
        sample_step = 1 / batch.sampling_frequency

        for env_index, env_descr in enumerate(batch.environments):
            model = mujoco.MjModel.from_xml_string(self._make_mjcf(env_descr))
            data = mujoco.MjData(model)

            initial_targets = [
                dof_state
                for posed_actor in env_descr.actors
                for dof_state in posed_actor.dof_states
            ]
            self._set_dof_targets(data, initial_targets)

            for posed_actor in env_descr.actors:
                posed_actor.dof_states

            if not self._headless:
                viewer = mujoco_viewer.MujocoViewer(
                    model,
                    data,
                )

            last_control_time = 0.0
            last_sample_time = 0.0

            # sample initial state
            # TODO

            while (time := data.time) < batch.simulation_time:
                # do control if it is time
                if time >= last_control_time + control_step:
                    control = ActorControl()
                    batch.control(env_index, control_step, control)
                    actor_targets = control._dof_targets
                    actor_targets.sort(key=lambda t: t[0])
                    targets = [
                        target
                        for actor_target in actor_targets
                        for target in actor_target[1]
                    ]
                    self._set_dof_targets(data, targets)

                # sample state if it is time
                if time >= last_sample_time + sample_step:
                    pass  # TODO

                # step simulation
                mujoco.mj_step(model, data)

                if not self._headless:
                    viewer.render()

            if not self._headless:
                viewer.close()

            # sample one final time
            # TODO

        raise NotImplementedError()

    @staticmethod
    def _make_mjcf(env_descr: Environment) -> str:
        assets = {}

        env_mjcf = mjcf.RootElement(model="environment")

        env_mjcf.compiler.angle = "radian"

        env_mjcf.option.timestep = 0.02
        env_mjcf.option.integrator = "RK4"

        env_mjcf.option.gravity = [0, 0, -9.81]

        env_mjcf.worldbody.add(
            "geom",
            name="ground",
            type="plane",
            size=[10, 10, 1],
            rgba=[0.2, 0.2, 0.2, 1],
        )
        env_mjcf.worldbody.add(
            "light",
            pos=[0, 0, 100],
            ambient=[0.5, 0.5, 0.5],
            directional=True,
            castshadow=False,
        )
        env_mjcf.visual.headlight.active = 0

        for actor_index, posed_actor in enumerate(env_descr.actors):
            urdf = physbot_to_urdf(
                posed_actor.actor,
                f"robot_{actor_index}",
                Vector3(),
                Quaternion(),
            )

            model = mujoco.MjModel.from_xml_string(urdf, assets)

            # mujoco can only save to a file, not directly to string,
            # so we create a temporary file.
            botfile = tempfile.NamedTemporaryFile(
                mode="r+", delete=False, suffix=".urdf"
            )
            mujoco.mj_saveLastXML(botfile.name, model)
            robot = mjcf.from_file(botfile)
            botfile.close()

            for joint in posed_actor.actor.joints:
                robot.actuator.add(
                    "position",
                    kp=1.0,
                    joint=robot.find(
                        namespace="joint",
                        identifier=joint.name,
                    ),
                    ctrllimited=True,
                    ctrlrange=[-joint.range, joint.range],
                    forcelimited=True,
                    forcerange=[-joint.effort, joint.effort],
                )
                robot.actuator.add(
                    "velocity",
                    kv=0.01,
                    joint=robot.find(namespace="joint", identifier=joint.name),
                )

            attachment_frame = env_mjcf.attach(robot)
            attachment_frame.add("freejoint")
            attachment_frame.pos = [
                posed_actor.position.x,
                posed_actor.position.y,
                posed_actor.position.z,
            ]

            attachment_frame.quat = [
                posed_actor.orientation.x,
                posed_actor.orientation.y,
                posed_actor.orientation.z,
                posed_actor.orientation.w,
            ]

        return env_mjcf.to_xml_string()

    @staticmethod
    def _set_dof_targets(data: mujoco.MjData, targets: List[float]) -> None:
        for i, target in enumerate(targets):
            data.ctrl[2 * i] = target
            data.ctrl[2 * i + 1] = 0
