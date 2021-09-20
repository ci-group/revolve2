from revolve2.abstract.creational.factory import Factory
from revolve2.simulation.simulator.adapter.evoman_simulator_adapter import EvomanSimulatorAdapter
from revolve2.simulation.simulator.adapter.gazebo_simulator_adapter import GazeboSimulatorAdapter
from revolve2.simulation.simulator.adapter.simulation_adapters import SimulatorAdapter
from revolve2.simulation.simulator.adapter.vrep.coppelia_simulator_adapter import CoppeliaSimulatorAdapter
from revolve2.simulation.simulator.simulator_command import SimulationRequest
from revolve2.simulation.simulator.simulator_type import SimulatorType
from test.simulation.simulator.test_connector_adapter import TestConnectorAdapter


class SimulatorFactory(Factory):

    def __init__(self, simulate_command: SimulationRequest):
        super().__init__()
        self.simulate_command = simulate_command

    def create(self) -> SimulatorAdapter:
        if self.simulate_command.ecosphere.simulator_type == SimulatorType.GAZEBO:
            return GazeboSimulatorAdapter(self.simulate_command.ecosphere)
        elif self.simulate_command.ecosphere.simulator_type == SimulatorType.COPPELIA:
            return CoppeliaSimulatorAdapter(self.simulate_command.ecosphere)
        elif self.simulate_command.ecosphere.simulator_type == SimulatorType.EVOMAN:
            return EvomanSimulatorAdapter(self.simulate_command.ecosphere)
        elif self.simulate_command.ecosphere.simulator_type == SimulatorType.NONE:
            return TestConnectorAdapter(self.simulate_command.ecosphere)
        elif self.simulate_command.ecosphere.simulator_type == SimulatorType.GENETIC:
            return None

        raise Exception("Unknown Simulator Type")

    def launch(self):
        pass
