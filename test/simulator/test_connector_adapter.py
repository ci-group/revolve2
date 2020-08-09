from revolve.evosphere.environment import Environment
from revolve.robot.robot import Robot

from simulation.measures import Measures
from simulation.simulator.simulation_connector import SimulatorConnector


class TestConnectorAdapter(SimulatorConnector):

    def __init__(self, environment: Environment):
        super().__init__(environment)
        self.simulator = False
        self.robot: Robot = None

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
        self.robot = None
        return True

    def simulate(self) -> Measures:
        return Measures().test()
