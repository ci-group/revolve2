from revolve.evosphere.environment import Environment
from simulation.measures import Measures
from simulation.simulator.simulation_connector import SimulatorConnector


class GazeboConnectorAdapter(SimulatorConnector):

    def __init__(self, environment: Environment):
        super().__init__(environment)

    def start_simulator(self) -> bool:
        return True

    def stop_simulator(self) -> bool:
        return True

    def add_robot(self, element: object):
        pass

    def remove_robot(self, element: object):
        pass

    def simulate(self) -> Measures:
        pass
