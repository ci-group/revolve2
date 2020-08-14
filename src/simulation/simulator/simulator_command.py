from nca.core.abstract.command import Command
from nca.core.agent.agents import Agents
from revolve.evosphere.world import Ecosphere
from simulation.simulator.simulator_helper import TaskPriority


class SimulateCommand(Command):

    def __init__(self, agents: Agents, ecosphere: Ecosphere, task_priority: TaskPriority = TaskPriority.LOW):
        self.agents: Agents = agents
        self.ecosphere: Ecosphere = ecosphere
        self.task_priority: TaskPriority = task_priority

    def __eq__(self, other):
        return self.agents == other.agents and self.ecosphere == other.ecosphere

    def __hash__(self):
        return hash((self.agents, self.ecosphere))
