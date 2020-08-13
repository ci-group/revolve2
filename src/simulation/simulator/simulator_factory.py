from simulation.simulator.simulator_command import SimulateCommand
from simulation.simulator.simulator_helper import SimulatorType
from simulation_test.simulator.test_connector_adapter import TestConnectorAdapter
from simulation.simulator.gazebo_connector_adapter import GazeboConnectorAdapter
from simulation.simulator.simulation_connector import SimulatorConnector


class SimulatorFactory:  #(Factory)

    def __init__(self, simulate_command: SimulateCommand):
        self.simulate_command = simulate_command

    def create(self) -> SimulatorConnector:
        if self.simulate_command.environment.simulator_type == SimulatorType.GAZEBO:
            return GazeboConnectorAdapter(self.simulate_command.environment)
        elif self.simulate_command.environment.simulator_type == SimulatorType.NONE:
            return TestConnectorAdapter(self.simulate_command.environment)