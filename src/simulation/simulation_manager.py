from typing import Dict

from evosphere.mock_ecosphere import MockEcosphere
from nca.core.abstract.configurations import SimulatorConfiguration
from revolve.evosphere.ecosphere import GeneticEcosphere, SimulationEcosphere
from simulation.simulation_measures import SimulationMeasures
from simulation.simulator.simulator_command import SimulateCommand
from simulation_test.simulator.mock_simulation_measures import MockSimulationMeasures
from src.simulation.simulation_supervisor import SimulationSupervisor


class SimulationManager:

    def __init__(self):
        self.configuration = SimulatorConfiguration()
        self.supervisors: Dict[SimulateCommand, SimulationSupervisor] = {}

    def simulate(self, request_command: SimulateCommand):

        if request_command not in self.supervisors.keys():
            self.supervisors[request_command] = SimulationSupervisor(request_command)

        for agent in request_command.agents:

            if isinstance(request_command.ecosphere, MockEcosphere):
                return MockSimulationMeasures()

            if isinstance(request_command.ecosphere, GeneticEcosphere):
                agent.fitness = request_command.ecosphere.fitness(agent)

            if isinstance(request_command.ecosphere, SimulationEcosphere):
                measures: SimulationMeasures = self.supervisors[request_command].work(agent, request_command)
                agent.performance(measures, request_command.ecosphere.fitness(measures))


    """
    def _find_available_supervisor(self):
        selected_supervisor = None

        for supervisor in self.supervisors:
            if selected_supervisor is None:
                selected_supervisor = supervisor
                continue

            if selected_supervisor.workload() > supervisor.workload():
                selected_supervisor = supervisor

        return selected_supervisor
    """