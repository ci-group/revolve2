from typing import Dict

from nca.core.abstract.configurations import SimulatorConfiguration
from nca.core.agent.agents import Agents
from simulation.simulator.simulator_helper import RequestCommand
from simulation.simulation_supervisor import SimulationSupervisor


class SimulationManager:

    def __init__(self):
        self.configuration = SimulatorConfiguration()
        self.supervisors: Dict[RequestCommand, SimulationSupervisor] = {}

    def simulate(self, robots: Agents, request_command: RequestCommand):

        if request_command not in self.supervisors.keys():
            self.supervisors[request_command] = SimulationSupervisor(request_command)

        for robot in robots:
            robot.measures = self.supervisors[request_command].work(robot)

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