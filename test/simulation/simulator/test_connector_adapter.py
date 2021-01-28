from revolve.evosphere.biosphere import Ecosphere
from revolve.robot.robot import Robot
from simulation.simulator.adapter.simulation_adapters import SimulatorAdapter

from src.simulation.simulation_measures import SimulationMeasures


class TestConnectorAdapter(SimulatorAdapter):

    def __init__(self, ecosphere: Ecosphere):
        super().__init__(ecosphere)
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
