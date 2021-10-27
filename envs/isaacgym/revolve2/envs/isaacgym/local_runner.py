import os
import tempfile
from dataclasses import dataclass
from typing import List, Mapping, Tuple

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
            env: gymapi.Env
            actors: Mapping[str, int]

        _gym: gymapi.Gym
        _batch: Batch

        _sim: gymapi.Sim
        _viewer: gymapi.Viewer
        _simulation_time: int
        _gymenvs: List[GymEnv]

        def __init__(self, gym: gymapi.Gym, batch: Batch):
            self._gym = gym
            self._batch = batch

            self._sim = self._create_sim()
            self._gymenvs = self._create_envs()
            self._viewer = self._create_viewer()

        def _create_sim(self) -> gymapi.Sim:
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

            sim = self._gym.create_sim(type=gymapi.SIM_PHYSX, params=sim_params)

            if sim is None:
                raise RuntimeError()

            return sim

        def _create_envs(self) -> List[GymEnv]:
            gymenvs: List[self.GymEnv] = []

            plane_params = gymapi.PlaneParams()
            plane_params.normal = gymapi.Vec3(0, 0, 1)
            plane_params.distance = 0
            plane_params.static_friction = 1
            plane_params.dynamic_friction = 1
            plane_params.restitution = 0
            self._gym.add_ground(self._sim, plane_params)

            for env_name, env_descr in self._batch.environments.items():
                env = self._gym.create_env(
                    self._sim,
                    gymapi.Vec3(-25.0, -25.0, 0.0),
                    gymapi.Vec3(25.0, 25.0, 25.0),
                    1,
                )

                gymenv = self.GymEnv(env, {})

                for actor_name, actor_descr in env_descr.actors.items():
                    botfile = tempfile.NamedTemporaryFile(
                        mode="r+", delete=False, suffix=".urdf"
                    )
                    botfile.writelines(
                        physbot_to_urdf(
                            actor_descr, f"robot_{actor_name}", Vector3(), Quaternion()
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
                    pose.p = gymapi.Vec3(0, 0, 0.5)
                    pose.r = gymapi.Quat(0, 0, 0.0, 0.707107)
                    actor_handle = self._gym.create_actor(
                        env, actor_asset, pose, f"robot_{actor_name}", 0, 0
                    )

                    props = self._gym.get_actor_dof_properties(env, actor_handle)
                    props["driveMode"].fill(gymapi.DOF_MODE_POS)
                    props["stiffness"].fill(1000.0)
                    props["damping"].fill(600.0)
                    self._gym.set_actor_dof_properties(env, actor_handle, props)

                    gymenv.actors[actor_name] = actor_handle

                gymenvs.append(gymenv)

            return gymenvs

        def _create_viewer(self) -> gymapi.Viewer:
            viewer = self._gym.create_viewer(self._sim, gymapi.CameraProperties())
            if viewer is None:
                raise RuntimeError()
            cam_pos = gymapi.Vec3(-4.0, -1.0, 4.0)
            cam_target = gymapi.Vec3(0.0, 0.0, 0.0)
            self._gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

            return viewer

        def run(self) -> List[Tuple[float, State]]:
            states: List[Tuple[float, State]] = []

            while (
                time := self._gym.get_sim_time(self._sim)
            ) < self._batch.simulation_time:
                if time % 0.2:  # TODO
                    control = ActorControl()
                    self._batch.control(0.2, control)

                    # TODO this is a temporary implementation just as an MVP
                    for env_index, (env_name, env) in enumerate(
                        self._batch.environments.items()
                    ):
                        if env_name in control._position_targets:
                            for actor_name, targets in control._position_targets[
                                env_name
                            ].items():
                                dof_dict = self._gym.get_actor_dof_dict(
                                    self._gymenvs[env_index].env,
                                    self._gymenvs[env_index].actors[actor_name],
                                )

                                targets_list = [None] * len(dof_dict)

                                for dof_name, dof_index in dof_dict.items():
                                    if dof_name not in targets:
                                        raise RuntimeError(
                                            "Not all targets are set for actor."
                                        )
                                    targets_list[dof_index] = targets[dof_name]

                                self._gym.set_actor_dof_position_targets(
                                    self._gymenvs[env_index].env,
                                    self._gymenvs[env_index].actors[actor_name],
                                    targets_list,
                                )

                self._gym.simulate(self._sim)
                self._gym.fetch_results(self._sim, True)
                self._gym.step_graphics(self._sim)
                self._gym.draw_viewer(self._viewer, self._sim, False)

                states.append((time, self._get_state()))

            return states

        def cleanup(self) -> None:
            self._gym.destroy_viewer(self._viewer)
            self._gym.destroy_sim(self._sim)

        def _get_state(self) -> State:
            state = State([])

            for gymenv in self._gymenvs:
                state.envs.append(EnvironmentState([]))
                for actor_index, _ in enumerate(gymenv.actors.items()):
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

    def __init__(self):
        self._gym = gymapi.acquire_gym()

    async def run_batch(self, batch: Batch) -> List[Tuple[float, State]]:
        simulator = self.Simulator(self._gym, batch)
        states = simulator.run()
        simulator.cleanup()

        return states
