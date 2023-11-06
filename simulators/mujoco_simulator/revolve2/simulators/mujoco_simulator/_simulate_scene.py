import logging
import math

import cv2
import mujoco
import mujoco_viewer
import numpy as np
import numpy.typing as npt

from revolve2.simulation.scene import Scene, SimulationState
from revolve2.simulation.simulator import RecordSettings

from ._control_interface_impl import ControlInterfaceImpl
from ._scene_to_model import scene_to_model
from ._simulation_state_impl import SimulationStateImpl


def simulate_scene(
    scene_id: int,
    scene: Scene,
    headless: bool,
    record_settings: RecordSettings | None,
    start_paused: bool,
    control_step: float,
    sample_step: float | None,
    simulation_time: int | None,
    simulation_timestep: float,
    cast_shadows: bool,
    fast_sim: bool,
) -> list[SimulationState]:
    """
    Simulate a scene.

    :param scene_id: An id for this scene, unique between all scenes ran in parallel.
    :param scene: The scene to simulate.
    :param headless: If False, a viewer will be opened that allows a user to manually view and manually interact with the simulation.
    :param record_settings: If not None, recording will be done according to these settings.
    :param start_paused: If true, the simulation will start in a paused state. Only makessense when headless is False.
    :param control_step: The time between each call to the handle function of the scene handler. In seconds.
    :param sample_step: The time between each state sample of the simulation. In seconds.
    :param simulation_time: How long to simulate for. In seconds.
    :param simulation_timestep: The duration to integrate over during each step of the simulation. In seconds.
    :param cast_shadows: If shadows are cast.
    :param fast_sim: If fancy rendering is disabled.
    :returns: The results of simulation. The number of returned states depends on `sample_step`.
    """
    logging.info(f"Simulating scene {scene_id}")

    model, mapping = scene_to_model(
        scene, simulation_timestep, cast_shadows=cast_shadows, fast_sim=fast_sim
    )
    data = mujoco.MjData(model)

    if not headless or record_settings is not None:
        viewer = mujoco_viewer.MujocoViewer(
            model,
            data,
        )
        viewer._render_every_frame = False  # Private but functionality is not exposed and for now it breaks nothing.
        viewer._paused = start_paused

    if record_settings is not None:
        video_step = 1 / record_settings.fps
        video_file_path = f"{record_settings.video_directory}/{scene_id}.mp4"
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

    # The measured states of the simulation
    simulation_states: list[SimulationState] = []

    # Compute forward dynamics without actually stepping forward in time.
    # This updates the data so we can read out the initial state.
    mujoco.mj_forward(model, data)

    # Sample initial state.
    if sample_step is not None:
        simulation_states.append(
            SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mapping)
        )

    control_interface = ControlInterfaceImpl(
        data=data, abstraction_to_mujoco_mapping=mapping
    )
    while (time := data.time) < (
        float("inf") if simulation_time is None else simulation_time
    ):
        # do control if it is time
        if time >= last_control_time + control_step:
            last_control_time = math.floor(time / control_step) * control_step

            simulation_state = SimulationStateImpl(
                data=data, abstraction_to_mujoco_mapping=mapping
            )
            scene.handler.handle(simulation_state, control_interface, control_step)

        # sample state if it is time
        if sample_step is not None:
            if time >= last_sample_time + sample_step:
                last_sample_time = int(time / sample_step) * sample_step
                simulation_states.append(
                    SimulationStateImpl(
                        data=data, abstraction_to_mujoco_mapping=mapping
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

    # Sample one final time.
    if sample_step is not None:
        simulation_states.append(
            SimulationStateImpl(data=data, abstraction_to_mujoco_mapping=mapping)
        )

    logging.info(f"Scene {scene_id} done.")

    return simulation_states
