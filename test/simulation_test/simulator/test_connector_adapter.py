from revolve.evosphere.environment import Environment
from revolve.robot.robot import Robot

from src.simulation.simulation_measures import SimulationMeasures
from src.simulation.simulator.simulation_connector import SimulatorConnector


class TestConnectorAdapter(SimulatorConnector):

    def __init__(self, environment: Environment):
        super().__init__(environment)
        self.simulator = False

    def start_simulator(self) -> bool:
        self.simulator = True
        return True

    def stop_simulator(self) -> bool:
        self.simulator = False
        return True

    def add_robot(self, robot: Robot):
        self.robot = robot

        return True

    def remove_robot(self, robot: Robot):
        if robot == self.robot:
            self.robot = None
        else:
            raise Exception("Robot does not exist")

        return True

    def simulate(self) -> SimulationMeasures:
        return SimulationMeasures().test()
