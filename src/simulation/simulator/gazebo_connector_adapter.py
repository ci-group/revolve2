from revolve.evosphere.biosphere import Ecosphere
from revolve.robot.robot import Robot
from src.simulation.simulation_measures import SimulationMeasures
from src.simulation.simulator.simulation_connector import RobotSimulatorConnector


class GazeboConnectorAdapter(RobotSimulatorConnector):

    def __init__(self, ecosphere: Ecosphere):
        super().__init__(ecosphere)

    def start_simulator(self) -> bool:
        return True

    def stop_simulator(self) -> bool:
        return True

    def _add_robot(self, robot: Robot):
        pass

    def _remove_robot(self, robot: Robot):
        pass

    def _simulate(self) -> SimulationMeasures:
        pass
