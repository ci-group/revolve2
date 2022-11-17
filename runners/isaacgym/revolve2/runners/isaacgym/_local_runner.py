import logging
import math
import multiprocessing as mp
import os
import tempfile
from dataclasses import dataclass
from typing import Callable, List, Optional

import colored
import numpy as np
from isaacgym import gymapi
from pyrr import Quaternion, Vector3
from revolve2.core.physics.actor import Actor
from revolve2.core.physics.actor.urdf import to_urdf as physbot_to_urdf
from revolve2.core.physics.running import (
    ActorControl,
    ActorState,
    Batch,
    BatchResults,
    EnvironmentResults,
    EnvironmentState,
    RecordSettings,
    Runner,
)

from .terrains import flat_terrain_generator


def default_sim_params() -> gymapi.SimParams:
    """
    Get default Isaac Gym parameters.

    :returns: The parameters.
    """
    sim_params = gymapi.SimParams()
    sim_params.dt = 0.02
    sim_params.substeps = 2
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)

    sim_params.physx.solver_type = 1
    sim_params.physx.num_position_iterations = 4
    sim_params.physx.num_velocity_iterations = 1
    sim_params.physx.num_threads = 1
    sim_params.physx.use_gpu = True

    return sim_params


class LocalRunner(Runner):
    """Runner for simulating using Isaac Gym."""

    class _Simulator:
        """Inner simulator implementation."""

        """
        The offset between multi environments in isaac gym.

        Sadly this must be set to 0, because the simulation shows slightly different results based on the location of the environment.
        Suspect that this is because of some floating point rounding error.
        So, for reproducability, all environments will be exactly at the origin.
        """
        ENV_SIZE = 0.0

        @dataclass
        class GymEnv:
            env: gymapi.Env  # environment handle
            actors: List[
                int
            ]  # actor handles, in same order as provided by environment description

        _real_time: bool

        _gym: gymapi.Gym
        _batch: Batch

        _sim: gymapi.Sim
        _viewer: Optional[gymapi.Viewer]
        _simulation_time: int
        _gymenvs: List[
            GymEnv
        ]  # environments, in same order as provided by batch description

        def __init__(
            self,
            batch: Batch,
            headless: bool,
            real_time: bool,
            max_gpu_contact_pairs: int,
            terrain_generator: Callable[[gymapi.Gym, gymapi.Sim], None],
            sim_params: gymapi.SimParams,
        ):
            self._gym = gymapi.acquire_gym()
            self._batch = batch

            sim_params.physx.max_gpu_contact_pairs = max_gpu_contact_pairs
            self._sim = self._create_sim(sim_params)
            self._gymenvs = self._create_envs(terrain_generator)

            if headless:
                self._viewer = None
            else:
                self._viewer = self._create_viewer()

            self._real_time = real_time

            self._gym.prepare_sim(self._sim)

        def _create_sim(self, sim_params: gymapi.SimParams) -> gymapi.Sim:
            sim = self._gym.create_sim(type=gymapi.SIM_PHYSX, params=sim_params)

            if sim is None:
                raise RuntimeError()

            return sim

        def _create_envs(
            self, terrain_generator: Callable[[gymapi.Gym, gymapi.Sim], None]
        ) -> List[GymEnv]:
            gymenvs: List[LocalRunner._Simulator.GymEnv] = []

            terrain_generator(self._gym, self._sim)

            num_per_row = int(math.sqrt(len(self._batch.environments)))

            for env_index, env_descr in enumerate(self._batch.environments):
                if len(env_descr.static_geometries) != 0:
                    raise NotImplementedError(
                        "Environment abstraction not supported for Isaac Gym runner."
                    )

                env = self._gym.create_env(
                    self._sim,
                    gymapi.Vec3(-self.ENV_SIZE, -self.ENV_SIZE, 0.0),
                    gymapi.Vec3(self.ENV_SIZE, self.ENV_SIZE, self.ENV_SIZE),
                    num_per_row,
                )

                gymenv = self.GymEnv(env, [])
                gymenvs.append(gymenv)

                for actor_index, posed_actor in enumerate(env_descr.actors):
                    # sadly isaac gym can only read robot descriptions from a file,
                    # so we create a temporary file.
                    with tempfile.NamedTemporaryFile(
                        mode="r+", delete=True, suffix="_isaacgym.urdf"
                    ) as botfile:
                        botfile.writelines(
                            physbot_to_urdf(
                                posed_actor.actor,
                                f"robot_{actor_index}",
                                Vector3(),
                                Quaternion(),
                            )
                        )
                        botfile.flush()
                        asset_root = os.path.dirname(botfile.name)
                        urdf_file = os.path.basename(botfile.name)
                        asset_options = gymapi.AssetOptions()
                        asset_options.angular_damping = 0.0
                        actor_asset = self._gym.load_urdf(
                            self._sim, asset_root, urdf_file, asset_options
                        )

                    if actor_asset is None:
                        raise RuntimeError()

                    pose = gymapi.Transform()
                    pose.p = gymapi.Vec3(
                        posed_actor.position.x,
                        posed_actor.position.y,
                        posed_actor.position.z,
                    )
                    pose.r = gymapi.Quat(
                        posed_actor.orientation.x,
                        posed_actor.orientation.y,
                        posed_actor.orientation.z,
                        posed_actor.orientation.w,
                    )

                    # create an aggregate for this robot
                    # disabling self collision to both improve performance and improve stability
                    num_bodies = self._gym.get_asset_rigid_body_count(actor_asset)
                    num_shapes = self._gym.get_asset_rigid_shape_count(actor_asset)
                    enable_self_collision = False
                    self._gym.begin_aggregate(
                        env, num_bodies, num_shapes, enable_self_collision
                    )

                    actor_handle: int = self._gym.create_actor(
                        env,
                        actor_asset,
                        pose,
                        f"robot_{actor_index}",
                        env_index,
                        0,
                    )
                    gymenv.actors.append(actor_handle)

                    self._gym.end_aggregate(env)

                    # TODO make all this configurable.
                    props = self._gym.get_actor_dof_properties(env, actor_handle)
                    props["driveMode"].fill(gymapi.DOF_MODE_POS)
                    props["stiffness"].fill(5.0)
                    props["damping"].fill(0.05)
                    self._gym.set_actor_dof_properties(env, actor_handle, props)

                    all_rigid_props = self._gym.get_actor_rigid_shape_properties(
                        env, actor_handle
                    )

                    for body, rigid_props in zip(
                        posed_actor.actor.bodies,
                        all_rigid_props,
                    ):
                        rigid_props.friction = body.static_friction
                        rigid_props.rolling_friction = body.dynamic_friction

                    self._gym.set_actor_rigid_shape_properties(
                        env, actor_handle, all_rigid_props
                    )

                    self.set_actor_dof_position_targets(
                        env, actor_handle, posed_actor.actor, posed_actor.dof_states
                    )
                    self.set_actor_dof_positions(
                        env, actor_handle, posed_actor.actor, posed_actor.dof_states
                    )

            return gymenvs

        def _create_viewer(self) -> gymapi.Viewer:
            # TODO provide some sensible default and make configurable
            viewer = self._gym.create_viewer(self._sim, gymapi.CameraProperties())
            if viewer is None:
                raise RuntimeError()
            num_per_row = math.sqrt(len(self._batch.environments))
            cam_pos = gymapi.Vec3(
                num_per_row / 2.0 - 0.5, num_per_row / 2.0 + 0.5, num_per_row
            )
            cam_target = gymapi.Vec3(
                num_per_row / 2.0 - 0.5, num_per_row / 2.0 + 0.5 - 1, 0.0
            )
            self._gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

            return viewer

        def run(self) -> BatchResults:
            results = BatchResults([EnvironmentResults([]) for _ in self._gymenvs])

            control_step = 1 / self._batch.control_frequency
            sample_step = 1 / self._batch.sampling_frequency

            last_control_time = 0.0
            last_sample_time = 0.0

            # sample initial state
            self._append_states(results, 0.0)

            while (
                time := self._gym.get_sim_time(self._sim)
            ) < self._batch.simulation_time:
                # do control if it is time
                if time >= last_control_time + control_step:
                    last_control_time = math.floor(time / control_step) * control_step
                    controls = [ActorControl() for _ in self._batch.environments]
                    for control, env in zip(controls, self._batch.environments):
                        env.controller.control(control_step, control)

                    dof_targets = [
                        (env_index, actor_index, targets)
                        for env_index, control in enumerate(controls)
                        for (actor_index, targets) in control._dof_targets
                    ]

                    for (env_index, actor_index, targets) in dof_targets:
                        env_handle = self._gymenvs[env_index].env
                        actor_handle = self._gymenvs[env_index].actors[actor_index]
                        actor = (
                            self._batch.environments[env_index]
                            .actors[actor_index]
                            .actor
                        )

                        self.set_actor_dof_position_targets(
                            env_handle, actor_handle, actor, targets
                        )

                # sample state if it is time
                if time >= last_sample_time + sample_step:
                    last_sample_time = int(time / sample_step) * sample_step
                    self._append_states(results, time)

                # step simulation
                self._gym.simulate(self._sim)
                self._gym.fetch_results(self._sim, True)

                if self._viewer is not None:
                    self._gym.step_graphics(self._sim)
                    self._gym.draw_viewer(self._viewer, self._sim, False)

                if self._real_time:
                    self._gym.sync_frame_time(self._sim)

            # sample one final time
            self._append_states(results, time)

            return results

        def set_actor_dof_position_targets(
            self,
            env_handle: gymapi.Env,
            actor_handle: int,
            actor: Actor,
            targets: List[float],
        ) -> None:
            if len(targets) != len(actor.joints):
                raise RuntimeError("Need to set a target for every dof")

            if not all(
                [
                    target >= -joint.range and target <= joint.range
                    for target, joint in zip(
                        targets,
                        actor.joints,
                    )
                ]
            ):
                raise RuntimeError("Dof targets must lie within the joints range.")

            self._gym.set_actor_dof_position_targets(
                env_handle,
                actor_handle,
                targets,
            )

        def set_actor_dof_positions(
            self,
            env_handle: gymapi.Env,
            actor_handle: int,
            actor: Actor,
            positions: List[float],
        ) -> None:
            num_dofs = len(actor.joints)

            if len(positions) != num_dofs:
                raise RuntimeError("Need to set a position for every dof")

            if num_dofs != 0:  # isaac gym does not understand zero length arrays...
                dof_states = np.zeros(num_dofs, dtype=gymapi.DofState.dtype)
                dof_positions = dof_states["pos"]

                for i in range(len(dof_positions)):
                    dof_positions[i] = positions[i]
                self._gym.set_actor_dof_states(
                    env_handle, actor_handle, dof_states, gymapi.STATE_POS
                )

        def cleanup(self) -> None:
            if self._viewer is not None:
                self._gym.destroy_viewer(self._viewer)
            self._gym.destroy_sim(self._sim)

        def _append_states(self, batch_results: BatchResults, time: float) -> None:
            for gymenv, environment_results in zip(
                self._gymenvs, batch_results.environment_results
            ):
                env_state = EnvironmentState(time, [])
                for actor_handle in gymenv.actors:
                    pose = self._gym.get_actor_rigid_body_states(
                        gymenv.env, actor_handle, gymapi.STATE_POS
                    )["pose"]
                    position = pose["p"][0]  # [0] is center of root element
                    orientation = pose["r"][0]
                    env_state.actor_states.append(
                        ActorState(
                            Vector3([position[0], position[1], position[2]]),
                            Quaternion(
                                [
                                    orientation[0],
                                    orientation[1],
                                    orientation[2],
                                    orientation[3],
                                ]
                            ),
                        )
                    )
                environment_results.environment_states.append(env_state)

    _sim_params: gymapi.SimParams
    _headless: bool
    _real_time: bool
    _max_gpu_contact_pairs: int
    _terrain_generator: Callable[[gymapi.Gym, gymapi.Sim], None]

    def __init__(
        self,
        headless: bool = False,
        real_time: bool = False,
        max_gpu_contact_pairs: int = 1048576,
        terrain_generator: Callable[
            [gymapi.Gym, gymapi.Sim], None
        ] = flat_terrain_generator,
        sim_params: gymapi.SimParams = default_sim_params(),
    ):
        """
        Initialize this object.

        :param headless: If True, the simulation will not be rendered. This drastically improves performance.
        :param real_time: If True, the simulation will run in real-time.
        :param max_gpu_contact_pairs: Maximum number of contacts that can be stored by Isaac Gym. If you get a warning similar to "The application needs to increase PxgDynamicsMemoryConfig::foundLostAggregatePairsCapacity" you need to increase this parameter.
        :param terrain_generator: Function to generate terrain.
        :param sim_params: Isaac Gym specific simulation parameters. Default parameters are provided using the `default_sim_params` method.
        """
        self._headless = headless
        self._real_time = real_time
        self._max_gpu_contact_pairs = max_gpu_contact_pairs
        self._terrain_generator = terrain_generator
        self._sim_params = sim_params

    async def run_batch(
        self, batch: Batch, record_settings: Optional[RecordSettings] = None
    ) -> BatchResults:
        """
        Run the provided batch by simulating each contained environment.

        :param batch: The batch to run.
        :param record_settings: Optional settings for recording the runnings. If None, no recording is made.
        :returns: List of simulation states in ascending order of time.
        :raises NotImplementedError: Video recording is not yet supported.
        """
        if record_settings is not None:
            raise NotImplementedError()  # TODO

        logging.info(
            "\n--- Begin Isaac Gym log ----------------------------------------------------------------------------"
        )

        # sadly we must run Isaac Gym in a subprocess, because it has some big memory leaks.
        result_queue: mp.Queue = mp.Queue()  # type: ignore # TODO
        process = mp.Process(
            target=self._run_batch_impl,
            args=(
                result_queue,
                batch,
                self._headless,
                self._real_time,
                self._max_gpu_contact_pairs,
                self._terrain_generator,
                self._sim_params,
            ),
        )
        process.start()
        environment_results = []
        # environment_results are sent seperately for every environment
        # because sending all at once is too big for the queue.
        # should be good enough for now.
        # if the program hangs here in the future,
        # improve the way the results are passed back to the parent program.
        while (environment_result := result_queue.get()) is not None:
            environment_results.append(environment_result)
        process.join()
        logging.info(
            colored.attr("reset")
            + "--- End Isaac Gym log ------------------------------------------------------------------------------\n"
        )
        return BatchResults(environment_results)

    @classmethod
    def _run_batch_impl(
        cls,
        result_queue: mp.Queue,  # type: ignore # TODO
        batch: Batch,
        headless: bool,
        real_time: bool,
        max_gpu_contact_pairs: int,
        terrain_generator: Callable[[gymapi.Gym, gymapi.Sim], None],
        sim_params: gymapi.SimParams,
    ) -> None:
        _Simulator = cls._Simulator(
            batch=batch,
            headless=headless,
            real_time=real_time,
            max_gpu_contact_pairs=max_gpu_contact_pairs,
            terrain_generator=terrain_generator,
            sim_params=sim_params,
        )
        batch_results = _Simulator.run()
        _Simulator.cleanup()
        for environment_results in batch_results.environment_results:
            result_queue.put(environment_results)
        result_queue.put(None)
