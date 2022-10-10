import math
import tempfile
from typing import List

import mujoco
import mujoco_viewer

try:
    import logging

    old_len = len(logging.root.handlers)

    from dm_control import mjcf

    new_len = len(logging.root.handlers)

    assert (
        old_len + 1 == new_len
    ), "dm_control not adding logging handler as expected. Maybe they fixed their annoying behaviour? https://github.com/deepmind/dm_control/issues/314https://github.com/deepmind/dm_control/issues/314"

    logging.root.removeHandler(logging.root.handlers[-1])
except Exception as e:
    print("Failed to fix absl logging bug", e)
    pass

from pyrr import Quaternion, Vector3
from revolve2.core.physics.actor.urdf import to_urdf as physbot_to_urdf
from revolve2.core.physics.running import (
    ActorControl,
    ActorState,
    Batch,
    BatchResults,
    Environment,
    EnvironmentResults,
    EnvironmentState,
    Runner,
)


class LocalRunner(Runner):
    """Runner for simulating using Mujoco."""

    _headless: bool

    def __init__(self, headless: bool = False):
        """
        Initialize this object.

        :param headless: If True, the simulation will not be rendered. This drastically improves performance.
        """
        self._headless = headless

    async def run_batch(self, batch: Batch) -> BatchResults:
        """
        Run the provided batch by simulating each contained environment.

        :param batch: The batch to run.
        :returns: List of simulation states in ascending order of time.
        """
        logging.info("Starting simulation batch with mujoco.")

        control_step = 1 / batch.control_frequency
        sample_step = 1 / batch.sampling_frequency

        results = BatchResults([EnvironmentResults([]) for _ in batch.environments])

        for env_index, env_descr in enumerate(batch.environments):
            logging.info(f"Environment {env_index} (of {len(batch.environments)})")

            xml_string = self._make_mjcf(env_descr)
            model = mujoco.MjModel.from_xml_string(xml_string)

            # TODO initial dof state
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
            results.environment_results[env_index].environment_states.append(
                EnvironmentState(0.0, self._get_actor_states(env_descr, data, model))
            )

            while (time := data.time) < batch.simulation_time:
                # do control if it is time
                if time >= last_control_time + control_step:
                    last_control_time = math.floor(time / control_step) * control_step
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
                    last_sample_time = int(time / sample_step) * sample_step
                    results.environment_results[env_index].environment_states.append(
                        EnvironmentState(
                            time, self._get_actor_states(env_descr, data, model)
                        )
                    )

                # step simulation
                mujoco.mj_step(model, data)

                if not self._headless:
                    viewer.render()

            if not self._headless:
                viewer.close()

            # sample one final time
            results.environment_results[env_index].environment_states.append(
                EnvironmentState(time, self._get_actor_states(env_descr, data, model))
            )

        logging.info("Finished batch.")

        return results

    @staticmethod
    def _make_mjcf(env_descr: Environment) -> str:
        env_mjcf = mjcf.RootElement(model="environment")

        env_mjcf.compiler.angle = "radian"

        env_mjcf.option.timestep = 0.0005
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
            ambient=[1.0, 1.0, 1.0],
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

            model = mujoco.MjModel.from_xml_string(urdf)

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
                    kp=5.0,
                    joint=robot.find(
                        namespace="joint",
                        identifier=joint.name,
                    ),
                )
                robot.actuator.add(
                    "velocity",
                    kv=0.05,
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

        xml = env_mjcf.to_xml_string()
        if not isinstance(xml, str):
            raise RuntimeError("Error generating mjcf xml.")

        return xml

    @classmethod
    def _get_actor_states(
        cls, env_descr: Environment, data: mujoco.MjData, model: mujoco.MjModel
    ) -> List[ActorState]:
        return [
            cls._get_actor_state(i, data, model) for i in range(len(env_descr.actors))
        ]

    @staticmethod
    def _get_actor_state(
        robot_index: int, data: mujoco.MjData, model: mujoco.MjModel
    ) -> ActorState:
        bodyid = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_BODY,
            f"robot_{robot_index}/",  # the slash is added by dm_control. ugly but deal with it
        )
        assert bodyid >= 0

        qindex = model.body_jntadr[bodyid]

        # explicitly copy because the Vector3 and Quaternion classes don't copy the underlying structure
        position = Vector3([n for n in data.qpos[qindex : qindex + 3]])
        orientation = Quaternion([n for n in data.qpos[qindex + 3 : qindex + 3 + 4]])

        #contacts = data.contact
        #for c in contacts:
        #    # TODO: check if c.geom1 or c.geom2 are the ground plane!
        #    # TODO: how to lookup an mjData object given its ID?
        #    pass

        return ActorState(position, orientation)

    @staticmethod
    def _set_dof_targets(data: mujoco.MjData, targets: List[float]) -> None:
        if len(targets) * 2 != len(data.ctrl):
            raise RuntimeError("Need to set a target for every dof")
        for i, target in enumerate(targets):
            data.ctrl[2 * i] = target
            data.ctrl[2 * i + 1] = 0
