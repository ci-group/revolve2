from nca.core.abstract.creational.factory import Factory
from simulation.simulator.simulator_command import SimulationRequest
from simulation.simulator.simulator_helper import SimulatorType
from simulation.simulator.vrep_connector_adapter import VREPConnectorAdapter
from test.simulation.simulator.test_connector_adapter import TestConnectorAdapter
from simulation.simulator.gazebo_connector_adapter import GazeboConnectorAdapter
from simulation.simulator.simulation_connector import SimulatorConnector


class SimulatorFactory(Factory):

    def __init__(self, simulate_command: SimulationRequest):
        super().__init__()
        self.simulate_command = simulate_command

    def create(self) -> SimulatorConnector:
        if self.simulate_command.ecosphere.simulator_type == SimulatorType.GAZEBO:
            return GazeboConnectorAdapter(self.simulate_command.ecosphere)
        elif self.simulate_command.ecosphere.simulator_type == SimulatorType.V_REP:
            return VREPConnectorAdapter(self.simulate_command.ecosphere)
        elif self.simulate_command.ecosphere.simulator_type == SimulatorType.NONE:
            return TestConnectorAdapter(self.simulate_command.ecosphere)
        elif self.simulate_command.ecosphere.simulator_type == SimulatorType.GENETIC:
            return None

        raise Exception("Unknown Simulator Type")
