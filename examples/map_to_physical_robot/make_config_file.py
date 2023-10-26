import pickle
from revolve2.modular_robot_physical import Config
from revolve2.ci_group.modular_robots_v1 import gecko_v1
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.ci_group import terrains
from revolve2.ci_group.simulation import make_standard_batch_parameters
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body import RightAngles
from revolve2.modular_robot.body.v1 import ActiveHingeV1, BodyV1, BrickV1
from revolve2.modular_robot.brain.cpg import BrainCpgNetworkNeighborRandom
from revolve2.modular_robot_physical import Config, UUIDKey
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator





def main() -> None:
    """Creating a Config for the physical robot."""
    rng = make_rng_time_seed()
    # creating a modular robot, as done in the simulate_single_robot example (this can be outsourced to EA).
    body = gecko_v1()
    brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)
    robot = ModularRobot(body, brain)

    """
    Creating a config multiple parameters are used.
    
    modular_robot: This is passing the ModularRobot object, for the physical robot to use.
    
    hinge_mapping: Here you have to map ActiveHinge objects from the ModularRobot to GPIO-pins on the physical robots HAT.
    
    run_duration: Set the desired length of the experiment in seconds.
    
    control_frequency: Determines how many control pulses the robot can do per second (The higher the smoother the movement, but also makes movements slower)
    
    initial_hinge_positions: Mapping the initial positions for the idle Robot. In Revolve2 the simulator defaults to 0.0.
    
    inverse_servos: Sometimes mounting servos on the physical robot can have an inverse direction as desired. 
        You dont have to change the physical servos orientation, but can instead parse the GPIO-pin as inversed (example: {13: True}).
    """
    config = Config(
        modular_robot=robot,
        hinge_mapping={},
        run_duration=10,
        control_frequency=10,
        initial_hinge_positions={
            UUIDKey(active_hinge): 0.0 for active_hinge in body.find_active_hinges()
        },
        inverse_servos={}
    )

    # dump the pickle file to load it onto the physical robot
    with open("config.pickle", "wb") as f:
        pickle.dump(config, f)


if __name__ == '__main__':
    main()
    