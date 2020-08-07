from nca.core.agent import Individual, Agents
from nca.core.abstract.configurations import SimulatorConfiguration
from revolve.evosphere.environment import Environment
from simulation import SimulationSupervisor


class Simulation:

    def __init__(self):
        self.configuration = SimulatorConfiguration()
        self.supervisor = SimulationSupervisor()

    def _evaluate_agent(self, individual: Individual, environment: Environment):
        self.supervisor.work(individual)

    def simulate(self, agents: Agents, environment: Environment):

        for individual in agents:
            self._evaluate_agent(individual, environment)
