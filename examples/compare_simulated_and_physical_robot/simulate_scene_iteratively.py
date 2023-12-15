import logging
import mujoco
from revolve2.simulation.scene import Scene

from revolve2.simulators.mujoco_simulator._control_interface_impl import ControlInterfaceImpl
from revolve2.simulators.mujoco_simulator._scene_to_model import scene_to_model
from revolve2.simulators.mujoco_simulator._simulation_state_impl import SimulationStateImpl

from custom_viewer import CustomMujocoViewer
from revolve2.ci_group.simulation import STANDARD_SIMULATION_TIMESTEP, STANDARD_CONTROL_FREQUENCY


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
    try:
        logging.info(f"To iterate over states press 'K'")
        logging.info(f"To stop the simulation do Ctrl-C")
        while True:
            # TODO: make control
            simulation_state = SimulationStateImpl(
                data=data, abstraction_to_mujoco_mapping=mapping
            )
            #control_interface.set_joint_hinge_position_target()
            scene.handler.handle(simulation_state, control_interface, STANDARD_CONTROL_FREQUENCY)
            # step simulation
            mujoco.mj_step(model, data)


            # render if not headless. also render when recording and if it time for a new video frame.
            viewer.render()
    except KeyboardInterrupt:
        pass
    finally:
        viewer.close()
        logging.info(f"Testing done.")
