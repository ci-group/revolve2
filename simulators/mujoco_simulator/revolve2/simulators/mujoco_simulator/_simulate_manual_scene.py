"""Iterative Scene Simulator."""

import logging

import mujoco
from revolve2.ci_group.simulation_parameters import (
    STANDARD_CONTROL_FREQUENCY,
    STANDARD_SIMULATION_TIMESTEP,
)
from revolve2.simulation.scene import Scene

from ._control_interface_impl import ControlInterfaceImpl
from ._render_backend import RenderBackend
from ._scene_to_model import scene_to_model
from ._simulation_state_impl import SimulationStateImpl
from .viewers import CustomMujocoViewer, CustomMujocoViewerMode


def simulate_manual_scene(
    scene: Scene,
    render_backend: RenderBackend = RenderBackend.GLFW,
) -> None:
    """
    Simulate a scene for checking if a robot was built correctly.

    :param scene: The scene to simulate.
    :param render_backend: The render backend.
    """
    logging.info("Simulating scene")

    model, mapping = scene_to_model(
        scene, STANDARD_SIMULATION_TIMESTEP, cast_shadows=False, fast_sim=False
    )
    data = mujoco.MjData(model)
    viewer = CustomMujocoViewer(
        model, data, mode=CustomMujocoViewerMode.MANUAL, backend=render_backend
    )

    # Compute forward dynamics without actually stepping forward in time.
    mujoco.mj_forward(model, data)

    control_interface = ControlInterfaceImpl(
        data=data,
        abstraction_to_mujoco_mapping=mapping,
    )

    """Here we set our values for cycling different positions."""
    prev_position: int = (
        0  # This is the initial idle position for all hinges (index of the positions).
    )
    positions: list[float] = [
        0.0,
        0.5,
        1.0,
        -0.5,
        -1.0,
    ]  # Those are the possible cycle positions we want.
    target, current = (
        0.0,
        0.0,
    )  # Here we can check whether we are currently on the correct position.
    try:
        while True:
            simulation_state = SimulationStateImpl(
                data=data, abstraction_to_mujoco_mapping=mapping, camera_views={}
            )
            scene.handler.handle(
                simulation_state, control_interface, STANDARD_CONTROL_FREQUENCY
            )
            # step simulation
            if target != current:
                """
                If we are not on the target we adjust our hinges accordingly.

                The stepsize of 0.0025 allows us to approximate the real robots movement better.
                If the simulated hinges are forced to the target location directly with big angles, the simulation detects instabilities and wont render, therefore we choose smaller steps.
                """
                step = 0.0025 if target > current else -0.0025
                for hinge in mapping.hinge_joint.keys():
                    control_interface.set_joint_hinge_position_target(
                        hinge.value, current + step
                    )
                current += step

            mujoco.mj_step(model, data)

            position = viewer.render()
            if position is not None and prev_position != position:
                prev_position = position
                target = positions[prev_position]

    except KeyboardInterrupt:
        """If we press ctrl-C this script will end with the finally clause."""
        pass
    finally:
        viewer.close_viewer()
        logging.info("Testing done.")
