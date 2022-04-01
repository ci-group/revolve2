import math
import multiprocessing as mp
import os
import tempfile
from dataclasses import dataclass
from typing import List, Optional, Tuple

from isaacgym import gymapi
from pyrr import Quaternion, Vector3

from revolve2.core.physics.actor.urdf import to_urdf as physbot_to_urdf
from revolve2.core.physics.running import (
    ActorControl,
    ActorState,
    Batch,
    EnvironmentState,
    Runner,
    State,
)


class LocalRunner(Runner):
    class _Simulator:
        ENV_SIZE = 0.5

        @dataclass
        class GymEnv:
            env: gymapi.Env  # environment handle
            actors: List[
                int
            ]  # actor handles, in same order as provided by environment description

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
            sim_params: gymapi.SimParams,
            headless: bool,
        ):
            self._gym = gymapi.acquire_gym()
            self._batch = batch

            self._sim = self._create_sim(sim_params)
            self._gymenvs = self._create_envs()

            if headless:
                self._viewer = None
            else:
                self._viewer = self._create_viewer()

            self._gym.prepare_sim(self._sim)

        def _create_sim(self, sim_params: gymapi.SimParams) -> gymapi.Sim:
            sim = self._gym.create_sim(type=gymapi.SIM_PHYSX, params=sim_params)

            if sim is None:
                raise RuntimeError()

            return sim

        def _create_envs(self) -> List[GymEnv]:
            gymenvs: List[LocalRunner._Simulator.GymEnv] = []

            # TODO this is only temporary. When we switch to the new isaac sim it should be easily possible to
            # let the user create static object, rendering the group plane redundant.
            # But for now we keep it because it's easy for our first test release.
            plane_params = gymapi.PlaneParams()
            plane_params.normal = gymapi.Vec3(0, 0, 1)
            plane_params.distance = 0
            plane_params.static_friction = 1.0
            plane_params.dynamic_friction = 1.0
            plane_params.restitution = 0
            self._gym.add_ground(self._sim, plane_params)

            num_per_row = int(math.sqrt(len(self._batch.environments)))

            for env_index, env_descr in enumerate(self._batch.environments):
                env = self._gym.create_env(
                    self._sim,
                    gymapi.Vec3(-self.ENV_SIZE, -self.ENV_SIZE, 0.0),
                    gymapi.Vec3(self.ENV_SIZE, self.ENV_SIZE, self.ENV_SIZE),
                    num_per_row,
                )

                gymenv = self.GymEnv(env, [])

                for actor_index, posed_actor in enumerate(env_descr.actors):
                    # sadly isaac gym can only read robot descriptions from a file,
                    # so we create a temporary file.
                    botfile = tempfile.NamedTemporaryFile(
                        mode="r+", delete=False, suffix=".urdf"
                    )
                    botfile.writelines(
                        physbot_to_urdf(
                            posed_actor.actor,
                            f"robot_{actor_index}",
                            Vector3(),
                            Quaternion(),
                        )
                    )
                    botfile.close()
                    asset_root = os.path.dirname(botfile.name)
                    urdf_file = os.path.basename(botfile.name)
                    actor_asset = self._gym.load_urdf(self._sim, asset_root, urdf_file)
                    os.remove(botfile.name)

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
                    actor_handle: int = self._gym.create_actor(
                        env, actor_asset, pose, f"robot_{actor_index}", env_index, 0
                    )

                    # TODO make all this configurable.
                    props = self._gym.get_actor_dof_properties(env, actor_handle)
                    props["driveMode"].fill(gymapi.DOF_MODE_POS)
                    props["stiffness"].fill(0.5)
                    props["damping"].fill(0.01)
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

                    gymenv.actors.append(actor_handle)

                gymenvs.append(gymenv)

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

        def run(self) -> List[Tuple[float, State]]:
            states: List[Tuple[float, State]] = []  # (time, state)

            control_step = 1 / self._batch.control_frequency
            sample_step = 1 / self._batch.sampling_frequency

            last_control_time = 0.0
            last_sample_time = 0.0

            # sample initial state
            states.append((0.0, self._get_state()))

            while (
                time := self._gym.get_sim_time(self._sim)
            ) < self._batch.simulation_time:
                # do control if it is time
                if time >= last_control_time + control_step:
                    last_control_time = math.floor(time / control_step) * control_step
                    control = ActorControl()
                    self._batch.control(0.2, control)

                    for (env_index, actor_index, targets) in control._dof_targets:
                        env_handle = self._gymenvs[env_index].env
                        actor_handle = self._gymenvs[env_index].actors[actor_index]

                        if len(targets) != len(
                            self._batch.environments[env_index]
                            .actors[actor_index]
                            .actor.joints
                        ):
                            raise RuntimeError("Need to set a target for every dof")
                        self._gym.set_actor_dof_position_targets(
                            env_handle,
                            actor_handle,
                            targets,
                        )

                # sample state if it is time
                if time >= last_sample_time + sample_step:
                    last_sample_time = int(time / sample_step) * sample_step
                    states.append((time, self._get_state()))

                # step simulation
                self._gym.simulate(self._sim)
                self._gym.fetch_results(self._sim, True)
                self._gym.step_graphics(self._sim)

                if self._viewer is not None:
                    self._gym.draw_viewer(self._viewer, self._sim, False)

            # sample one final time
            states.append((time, self._get_state()))

            return states

        def cleanup(self) -> None:
            if self._viewer is not None:
                self._gym.destroy_viewer(self._viewer)
            self._gym.destroy_sim(self._sim)

        def _get_state(self) -> State:
            state = State([])

            for gymenv in self._gymenvs:
                env_state = EnvironmentState([])
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
                state.envs.append(env_state)

            return state

    _sim_params: gymapi.SimParams
    _headless: bool

    def __init__(self, sim_params: gymapi.SimParams, headless: bool = False):
        self._sim_params = sim_params
        self._headless = headless

    @staticmethod
    def SimParams() -> gymapi.SimParams:
        sim_params = gymapi.SimParams()
        sim_params.dt = 0.02
        sim_params.substeps = 2
        sim_params.up_axis = gymapi.UP_AXIS_Z
        sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)

        sim_params.physx.solver_type = 1
        sim_params.physx.num_position_iterations = 4
        sim_params.physx.num_velocity_iterations = 1
        sim_params.physx.num_threads = 1
        sim_params.physx.use_gpu = False

        return sim_params

    async def run_batch(self, batch: Batch) -> List[Tuple[float, State]]:
        # sadly we must run Isaac Gym in a subprocess, because it has some big memory leaks.
        result_queue: mp.Queue = mp.Queue()  # type: ignore # TODO
        process = mp.Process(
            target=self._run_batch_impl,
            args=(result_queue, batch, self._sim_params, self._headless),
        )
        process.start()
        states = []
        # states are sent state by state(every sample)
        # because sending all at once is too big for the queue.
        # should be good enough for now.
        # if the program hangs here in the future,
        # improve the way the results are passed back to the parent program.
        while (state := result_queue.get()) is not None:
            states.append(state)
        process.join()
        return states

    @classmethod
    def _run_batch_impl(
        cls,
        result_queue: mp.Queue,  # type: ignore # TODO
        batch: Batch,
        sim_params: gymapi.SimParams,
        headless: bool,
    ) -> None:
        _Simulator = cls._Simulator(batch, sim_params, headless)
        states = _Simulator.run()
        _Simulator.cleanup()
        for state in states:
            result_queue.put(state)
        result_queue.put(None)
