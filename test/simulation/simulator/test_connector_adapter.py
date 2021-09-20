import time
import numpy as np

from revolve2.nca.core.actor.individual import Individual
from revolve2.revolve.evosphere.biosphere import Ecosphere
from revolve2.revolve.robot.robot import Robot
from revolve2.simulation.simulator.adapter.simulation_adapters import SimulatorAdapter

from src.simulation.simulation_measures import SimulationMeasures


class TestConnectorAdapter(SimulatorAdapter):

    def _connect(self) -> bool:
        return True

    def _disconnect(self) -> bool:
        return True

    def _pause_simulation(self) -> bool:
        return True

    def _start_simulation(self) -> bool:
        return True

    def _stop_simulation(self) -> bool:
        return True

    def get_simulation_time(self):
        pass

    def execute(self, individual: Individual) -> SimulationMeasures:
        time.sleep(np.random.random()*3)
        return SimulationMeasures()

    def __init__(self, ecosphere: Ecosphere):
        super().__init__(ecosphere)
        self.simulator = False

    def _start_simulator(self) -> bool:
        self.simulator = True
        return True

    def _stop_simulator(self) -> bool:
        self.simulator = False
        return True

    def _add_robot(self, robot: Robot):
        self.robot = robot

        return True

    def _remove_robot(self, robot: Robot):
        if robot == self.robot:
            self.robot = None
        else:
            raise Exception("Robot does not exist")

        return True

    def _simulate(self) -> SimulationMeasures:
        return SimulationMeasures().test()
