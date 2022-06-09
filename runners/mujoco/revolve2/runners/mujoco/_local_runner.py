import math
import multiprocessing as mp
import os
import tempfile
from dataclasses import dataclass
from typing import List, Optional

import mujoco
import mujoco_viewer
import numpy as np
from pyrr import Quaternion, Vector3

from revolve2.core.physics.actor import Actor
from revolve2.core.physics.actor.urdf import to_urdf as physbot_to_urdf
from revolve2.core.physics.running import (
    ActorControl,
    ActorState,
    Batch,
    BatchResults,
    EnvironmentState,
    Runner,
)


class LocalRunner(Runner):
    _headless: bool

    def __init__(self, headless: bool = False):
        self._headless = headless

    async def run_batch(self, batch: Batch) -> BatchResults:
        assets = {}

        for env_index, env_descr in enumerate(batch.environments):
            for actor_index, posed_actor in enumerate(env_descr.actors):
                urdf = physbot_to_urdf(
                    posed_actor.actor,
                    f"robot_{actor_index}",
                    Vector3(),
                    Quaternion(),
                )

                model = mujoco.MjModel.from_xml_string(urdf, assets)
                data = mujoco.MjData(model)

                viewer = mujoco_viewer.MujocoViewer(model, data)

                for _ in range(100000):
                    mujoco.mj_step(model, data)
                    viewer.render()

                viewer.close()

                break
            break

        raise NotImplementedError()
