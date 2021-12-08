import os
import tempfile
from dataclasses import dataclass
from typing import List, Optional, Tuple

from pyrr import Quaternion, Vector3
from revolve2.core.physics.actor.urdf import to_urdf as physbot_to_urdf
from revolve2.core.physics.env import (
    ActorControl,
    ActorState,
    Batch,
    EnvironmentState,
    Runner,
    State,
)

from isaacgym import gymapi


class LocalRunner(Runner):
    class Simulator:
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
            gym: gymapi.Gym,
            batch: Batch,
            sim_params: gymapi.SimParams,
            headless: bool,
        ):
            self._gym = gym
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
            gymenvs: List[self.GymEnv] = []

            # TODO this is only temporary. When we switch to the new isaac sim it should be easily possible to
            # let the user create static object, rendering the group plane redundant.
            # But for now we keep it because it's easy for our first test release.
            plane_params = gymapi.PlaneParams()
            plane_params.normal = gymapi.Vec3(0, 0, 1)
            plane_params.distance = 0
            plane_params.static_friction = 1
            plane_params.dynamic_friction = 1
            plane_params.restitution = 0
            self._gym.add_ground(self._sim, plane_params)

            for env_descr in self._batch.environments:
                env = self._gym.create_env(
                    self._sim,
                    gymapi.Vec3(-25.0, -25.0, 0.0),  # TODO make these configurable
                    gymapi.Vec3(25.0, 25.0, 25.0),
                    1,
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
                    actor_handle = self._gym.create_actor(
                        env, actor_asset, pose, f"robot_{actor_index}", 0, 0
                    )

                    # TODO make all this configurable.
                    props = self._gym.get_actor_dof_properties(env, actor_handle)
                    props["driveMode"].fill(gymapi.DOF_MODE_POS)
                    props["stiffness"].fill(1000.0)
                    props["damping"].fill(600.0)
                    self._gym.set_actor_dof_properties(env, actor_handle, props)

                    gymenv.actors.append(actor_handle)

                gymenvs.append(gymenv)

            return gymenvs

        def _create_viewer(self) -> gymapi.Viewer:
            # TODO provide some sensible default and make configurable
            viewer = self._gym.create_viewer(self._sim, gymapi.CameraProperties())
            if viewer is None:
                raise RuntimeError()
            cam_pos = gymapi.Vec3(-4.0, -1.0, 4.0)
            cam_target = gymapi.Vec3(0.0, 0.0, 0.0)
            self._gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

            return viewer

        def run(self) -> List[Tuple[float, State]]:
            states: List[Tuple[float, State]] = []  # (time, state)

            control_step = 1 / self._batch.control_frequency
            sample_step = 1 / self._batch.sampling_frequency

            last_control_time = 0
            last_sample_time = 0

            while (
                time := self._gym.get_sim_time(self._sim)
            ) < self._batch.simulation_time:
                # do control if it is time
                if time >= last_control_time + control_step:
                    last_control_time = int(time / control_step) * control_step
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
                state.envs.append(EnvironmentState([]))
                for actor_index, _ in enumerate(gymenv.actors):
                    pose = self._gym.get_actor_rigid_body_states(
                        gymenv.env, actor_index, gymapi.STATE_POS
                    )["pose"]
                    position = pose["p"][0]  # [0] is center of root element
                    orientation = pose["r"][0]  # -> [0] is rotation of root element
                    state.envs[-1].actor_states.append(
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

            return state

    _gym = gymapi.Gym
    _sim_params: gymapi.SimParams

    def __init__(self, sim_params: gymapi.SimParams, headless: bool = False):
        self._gym = gymapi.acquire_gym()
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
        simulator = self.Simulator(self._gym, batch, self._sim_params, self._headless)
        states = simulator.run()  # TODO this is not async
        simulator.cleanup()

        return states
