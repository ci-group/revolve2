from typing import Dict

from nca.core.abstract.configurations import SimulatorConfiguration
from nca.core.agent.agents import Agents
from simulation.simulator.simulator_command import SimulateCommand
from src.simulation.simulation_supervisor import SimulationSupervisor


class SimulationManager:

    def __init__(self):
        self.configuration = SimulatorConfiguration()
        self.supervisors: Dict[SimulateCommand, SimulationSupervisor] = {}

    def simulate(self, request_command: SimulateCommand) -> Agents:

        if request_command not in self.supervisors.keys():
            self.supervisors[request_command] = SimulationSupervisor(request_command)

        for agent in request_command.agents:
            agent.measures = self.supervisors[request_command].work(agent, request_command)

        return request_command.agents

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