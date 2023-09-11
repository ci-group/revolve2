import concurrent.futures
import math
import os
import tempfile

import cv2
import mujoco
import mujoco_viewer
import numpy as np
import numpy.typing as npt

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
from revolve2.simulation.actor.urdf import to_urdf as physbot_to_urdf
from revolve2.simulation.running import (
    ActorControl,
    ActorState,
    Batch,
    BatchResults,
    Environment,
    EnvironmentResults,
    EnvironmentState,
    RecordSettings,
    Runner,
    geometry,
)


class LocalRunner(Runner):
    """Runner for simulating using Mujoco."""

    _headless: bool
    _start_paused: bool
    _num_simulators: int

    def __init__(
        self,
        headless: bool = False,
        start_paused: bool = False,
        num_simulators: int = 1,
    ):
        """
        Initialize this object.

        :param headless: If True, the simulation will not be rendered. This drastically improves performance.
        :param start_paused: If True, start the simulation paused. Only possible when not in headless mode.
        :param num_simulators: The number of simulators to deploy in parallel. They will take one core each but will share space on the main python thread for calculating control.
        """
        assert (
            headless or num_simulators == 1
        ), "Cannot have parallel simulators when visualizing."

        assert not (
            headless and start_paused
        ), "Cannot start simulation paused in headless mode."

        self._headless = headless
        self._start_paused = start_paused
        self._num_simulators = num_simulators

    @classmethod
    def _run_environment(
        cls,
        env_index: int,
        env_descr: Environment,
        headless: bool,
        record_settings: RecordSettings | None,
        start_paused: bool,
        control_step: float,
        sample_step: float,
        simulation_time: int | None,
        simulation_timestep: float,
    ) -> EnvironmentResults:
        logging.info(f"Environment {env_index}")

        model = cls._make_model(env_descr, simulation_timestep)

        data = mujoco.MjData(model)

        initial_targets = [
            dof_state
            for posed_actor in env_descr.actors
            for dof_state in posed_actor.dof_states
        ]
        cls._set_dof_targets(data, initial_targets)

        for posed_actor in env_descr.actors:
            posed_actor.dof_states

        if not headless or record_settings is not None:
            viewer = mujoco_viewer.MujocoViewer(
                model,
                data,
            )
            viewer._render_every_frame = False  # Private but functionality is not exposed and for now it breaks nothing.
            viewer._paused = start_paused

        if record_settings is not None:
            video_step = 1 / record_settings.fps
            video_file_path = f"{record_settings.video_directory}/{env_index}.mp4"
            fourcc = cv2.VideoWriter.fourcc(*"mp4v")
            video = cv2.VideoWriter(
                video_file_path,
                fourcc,
                record_settings.fps,
                (viewer.viewport.width, viewer.viewport.height),
            )

            viewer._hide_menu = True

        last_control_time = 0.0
        last_sample_time = 0.0
        last_video_time = 0.0  # time at which last video frame was saved

        results = EnvironmentResults([])

        # sample initial state
        results.environment_states.append(
            EnvironmentState(0.0, cls._get_actor_states(env_descr, data, model))
        )

        while (time := data.time) < (
            float("inf") if simulation_time is None else simulation_time
        ):
            # do control if it is time
            if time >= last_control_time + control_step:
                last_control_time = math.floor(time / control_step) * control_step
                control_user = ActorControl()
                env_descr.controller.control(control_step, control_user)
                actor_targets = control_user._dof_targets
                actor_targets.sort(key=lambda t: t[0])
                targets = [
                    target
                    for actor_target in actor_targets
                    for target in actor_target[1]
                ]
                cls._set_dof_targets(data, targets)

            # sample state if it is time
            if time >= last_sample_time + sample_step:
                last_sample_time = int(time / sample_step) * sample_step
                results.environment_states.append(
                    EnvironmentState(
                        time, cls._get_actor_states(env_descr, data, model)
                    )
                )

            # step simulation
            mujoco.mj_step(model, data)

            # render if not headless. also render when recording and if it time for a new video frame.
            if not headless or (
                record_settings is not None and time >= last_video_time + video_step
            ):
                viewer.render()

            # capture video frame if it's time
            if record_settings is not None and time >= last_video_time + video_step:
                last_video_time = int(time / video_step) * video_step

                # https://github.com/deepmind/mujoco/issues/285 (see also record.cc)
                img: npt.NDArray[np.uint8] = np.empty(
                    (viewer.viewport.height, viewer.viewport.width, 3),
                    dtype=np.uint8,
                )

                mujoco.mjr_readPixels(
                    rgb=img,
                    depth=None,
                    viewport=viewer.viewport,
                    con=viewer.ctx,
                )
                img = np.flip(img, axis=0)  # img is upside down initially
                video.write(img)

        if not headless or record_settings is not None:
            viewer.close()

        if record_settings is not None:
            video.release()

        # sample one final time
        results.environment_states.append(
            EnvironmentState(time, cls._get_actor_states(env_descr, data, model))
        )

        return results

    async def run_batch(
        self, batch: Batch, record_settings: RecordSettings | None = None
    ) -> BatchResults:
        """
        Run the provided batch by simulating each contained environment.

        :param batch: The batch to run.
        :param record_settings: Optional settings for recording the runnings. If None, no recording is made.
        :returns: List of simulation states in ascending order of time.
        """
        logging.info("Starting simulation batch with mujoco.")

        control_step = 1 / batch.control_frequency
        sample_step = 1 / batch.sampling_frequency

        if record_settings is not None:
            os.makedirs(record_settings.video_directory, exist_ok=False)

        with concurrent.futures.ProcessPoolExecutor(
            max_workers=self._num_simulators
        ) as executor:
            futures = [
                executor.submit(
                    self._run_environment,
                    env_index,
                    env_descr,
                    self._headless,
                    record_settings,
                    self._start_paused,
                    control_step,
                    sample_step,
                    batch.simulation_time,
                    batch.simulation_timestep,
                )
                for env_index, env_descr in enumerate(batch.environments)
            ]
            results = BatchResults([future.result() for future in futures])

        logging.info("Finished batch.")

        return results

    @staticmethod
    def _make_model(
        env_descr: Environment, simulation_timestep: float = 0.001
    ) -> mujoco.MjModel:
        env_mjcf = mjcf.RootElement(model="environment")

        env_mjcf.compiler.angle = "radian"

        env_mjcf.option.timestep = simulation_timestep
        env_mjcf.option.integrator = "RK4"

        env_mjcf.option.gravity = [0, 0, -9.81]

        heightmaps: list[geometry.Heightmap] = []
        for geo in env_descr.static_geometries:
            if isinstance(geo, geometry.Plane):
                env_mjcf.worldbody.add(
                    "geom",
                    type="plane",
                    pos=[geo.position.x, geo.position.y, geo.position.z],
                    size=[geo.size.x / 2.0, geo.size.y / 2.0, 1.0],
                    rgba=[geo.color.x, geo.color.y, geo.color.z, 1.0],
                )
            elif isinstance(geo, geometry.Heightmap):
                env_mjcf.asset.add(
                    "hfield",
                    name=f"hfield_{len(heightmaps)}",
                    nrow=len(geo.heights),
                    ncol=len(geo.heights[0]),
                    size=[geo.size.x, geo.size.y, geo.size.z, geo.base_thickness],
                )

                env_mjcf.worldbody.add(
                    "geom",
                    type="hfield",
                    hfield=f"hfield_{len(heightmaps)}",
                    pos=[geo.position.x, geo.position.y, geo.position.z],
                    quat=[
                        geo.orientation.x,
                        geo.orientation.y,
                        geo.orientation.z,
                        geo.orientation.w,
                    ],
                    # size=[geo.size.x, geo.size.y, 1.0],
                    rgba=[geo.color.x, geo.color.y, geo.color.z, 1.0],
                )
                heightmaps.append(geo)
            else:
                raise NotImplementedError()

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

            model = mujoco.MjModel.from_xml_string(urdf)

            # mujoco can only save to a file, not directly to string,
            # so we create a temporary file.
            try:
                with tempfile.NamedTemporaryFile(
                    mode="r+", delete=True, suffix="_mujoco.urdf"
                ) as botfile:
                    mujoco.mj_saveLastXML(botfile.name, model)
                    robot = mjcf.from_file(botfile)
            # handle an exception when the xml saving fails, it's almost certain to occur on Windows
            # since NamedTemporaryFile can't be opened twice when the file is still open.
            except Exception as e:
                print(repr(e))
                print(
                    "Setting 'delete' parameter to False so that the xml can be saved"
                )
                with tempfile.NamedTemporaryFile(
                    mode="r+", delete=False, suffix="_mujoco.urdf"
                ) as botfile:
                    # to make sure the temp file is always deleted,
                    # an error catching is needed, in case the xml saving fails and crashes the program
                    try:
                        mujoco.mj_saveLastXML(botfile.name, model)
                        robot = mjcf.from_file(botfile)
                        # On Windows, an open file can’t be deleted, and hence it has to be closed first before removing
                        botfile.close()
                        os.remove(botfile.name)
                    except Exception as e:
                        print(repr(e))
                        # On Windows, an open file can’t be deleted, and hence it has to be closed first before removing
                        botfile.close()
                        os.remove(botfile.name)

            for joint in posed_actor.actor.joints:
                # Add rotor inertia to joints. This value is arbitrarily chosen and appears stable enough.
                # Fine-tuning the armature value might be needed later.
                robot.find(namespace="joint", identifier=joint.name).armature = "0.002"
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

            # in mjcf w is first, not last.
            attachment_frame.quat = [
                posed_actor.orientation.w,
                posed_actor.orientation.x,
                posed_actor.orientation.y,
                posed_actor.orientation.z,
            ]

        xml = env_mjcf.to_xml_string()
        if not isinstance(xml, str):
            raise RuntimeError("Error generating mjcf xml.")

        model = mujoco.MjModel.from_xml_string(xml)

        # set height map values
        offset = 0

        for heightmap in heightmaps:
            for x in range(len(heightmap.heights)):
                for y in range(len(heightmap.heights[0])):
                    model.hfield_data[
                        y * len(heightmap.heights) + x
                    ] = heightmap.heights[x][y]
            offset += len(heightmap.heights) * len(heightmap.heights[0])

        return model

    @classmethod
    def _get_actor_states(
        cls, env_descr: Environment, data: mujoco.MjData, model: mujoco.MjModel
    ) -> list[ActorState]:
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

        return ActorState(position, orientation)

    @staticmethod
    def _set_dof_targets(data: mujoco.MjData, targets: list[float]) -> None:
        if len(targets) * 2 != len(data.ctrl):
            raise RuntimeError("Need to set a target for every dof")
        for i, target in enumerate(targets):
            data.ctrl[2 * i] = target
            data.ctrl[2 * i + 1] = 0
