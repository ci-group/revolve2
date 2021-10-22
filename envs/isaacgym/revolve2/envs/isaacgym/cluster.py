import os
import tempfile

from pyrr import Quaternion, Vector3
from revolve2.core.physics_robot.urdf import to_urdf as physbot_to_urdf

from isaacgym import gymapi

from .batch import Batch


class Cluster:
    async def run_batch(self, batch: Batch):  # TODO result
        gym = gymapi.acquire_gym()

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

        sim = gym.create_sim(type=gymapi.SIM_PHYSX, params=sim_params)

        if sim is None:
            raise RuntimeError()

        plane_params = gymapi.PlaneParams()
        plane_params.normal = gymapi.Vec3(0, 0, 1)
        plane_params.distance = 0
        plane_params.static_friction = 1
        plane_params.dynamic_friction = 1
        plane_params.restitution = 0
        gym.add_ground(sim, plane_params)

        viewer = gym.create_viewer(sim, gymapi.CameraProperties())
        if viewer is None:
            raise RuntimeError()
        cam_pos = gymapi.Vec3(-4.0, -1.0, 4.0)
        cam_target = gymapi.Vec3(0.0, 0.0, 0.0)
        gym.viewer_camera_look_at(viewer, None, cam_pos, cam_target)

        for i, val in enumerate(batch.environments):
            env = gym.create_env(
                sim,
                gymapi.Vec3(-25.0, -25.0, 0.0),
                gymapi.Vec3(25.0, 25.0, 25.0),
                1,
            )

            if val.robot is None:
                raise RuntimeError("Robot cannot be None")
            botfile = tempfile.NamedTemporaryFile(
                mode="r+", delete=False, suffix=".urdf"
            )
            botfile.writelines(
                physbot_to_urdf(val.robot, "robot", Vector3(), Quaternion())
            )
            botfile.close()
            asset_root = os.path.dirname(botfile.name)
            urdf_file = os.path.basename(botfile.name)
            robot_asset = gym.load_urdf(sim, asset_root, urdf_file)
            os.remove(botfile.name)

            if robot_asset is None:
                raise RuntimeError()

            pose = gymapi.Transform()
            pose.p = gymapi.Vec3(0, 0, 0.5)
            pose.r = gymapi.Quat(0, 0, 0.0, 0.707107)
            robot_handle = gym.create_actor(env, robot_asset, pose, "robot", 0, 0)

        while True:  # (time := gym.get_sim_time(sim)) < 10.0:
            # print(time)
            gym.simulate(sim)
            gym.fetch_results(sim, True)
            gym.step_graphics(sim)
            gym.draw_viewer(viewer, sim, False)

        gym.destroy_viewer(viewer)
        gym.destroy_sim(sim)
