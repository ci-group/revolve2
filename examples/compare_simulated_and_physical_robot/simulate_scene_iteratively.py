import logging
import mujoco
from revolve2.simulation.scene import Scene

from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl

from custom_viewer import CustomMujocoViewer
from revolve2.ci_group.simulation import STANDARD_SIMULATION_TIMESTEP, STANDARD_CONTROL_FREQUENCY
from revolve2.modular_robot.body.base import ActiveHinge


def simulate_scene_iteratively(
    scene: Scene,
) -> None:
    """
    Simulate a scene for checking if a robot was built correctly.

    :param scene: The scene to simulate.
    """
    logging.info(f"Simulating scene")

    model, mapping = scene_to_model(
        scene, STANDARD_SIMULATION_TIMESTEP, cast_shadows=False, fast_sim=False
    )
    data = mujoco.MjData(model)
    viewer = CustomMujocoViewer(
        model,
        data,
    )
    viewer._render_every_frame = False  # Private but functionality is not exposed and for now it breaks nothing.

    # Compute forward dynamics without actually stepping forward in time.
    # This updates the data so we can read out the initial state.
    mujoco.mj_forward(model, data)

    control_interface = ControlInterfaceImpl(
        data=data, abstraction_to_mujoco_mapping=mapping
    )
    prev_position = 0
    positions = [0.0, 0.5, 1.0, -0.5, -1.0]
    target, current = 0.0,  0.0
    try:
        logging.info(f"To iterate over states press 'K'")
        logging.info(f"To stop the simulation do Ctrl-C")
        while True:
            simulation_state = SimulationStateImpl(
                data=data, abstraction_to_mujoco_mapping=mapping
            )
            scene.handler.handle(simulation_state, control_interface, STANDARD_CONTROL_FREQUENCY)
            # step simulation
            if target != current:
                step = 0.01 if target > current else -0.01
                for hinge in mapping.hinge_joint.keys():
                    control_interface.set_joint_hinge_position_target(hinge.value, current + step)

            mujoco.mj_step(model, data)

            position = viewer.render()
            if prev_position != position:
                prev_position = position
                target = positions[prev_position]

    except KeyboardInterrupt:
        pass
    finally:
        viewer.close()
        logging.info(f"Testing done.")
