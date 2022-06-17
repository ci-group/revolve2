import tempfile

import mujoco
import mujoco_viewer
from pyrr import Quaternion, Vector3

from revolve2.core.physics.actor.urdf import to_urdf as physbot_to_urdf
from revolve2.core.physics.running import (
    ActorControl,
    ActorState,
    Batch,
    BatchResults,
    EnvironmentState,
    Runner,
)
from dm_control import mjcf


class LocalRunner(Runner):
    _headless: bool

    def __init__(self, headless: bool = False):
        self._headless = headless

    async def run_batch(self, batch: Batch) -> BatchResults:
        assets = {}

        for env_index, env_descr in enumerate(batch.environments):
            env_mjcf = mjcf.RootElement(model="environment")
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

            model = mujoco.MjModel.from_xml_string(env_mjcf.to_xml_string())
            data = mujoco.MjData(model)

            viewer = mujoco_viewer.MujocoViewer(model, data)

            for _ in range(100000):
                mujoco.mj_step(model, data)
                viewer.render()

            viewer.close()

            break

        raise NotImplementedError()
