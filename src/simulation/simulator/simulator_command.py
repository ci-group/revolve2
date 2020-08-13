from nca.core.abstract.command import Command
from nca.core.agent.agents import Agents
from simulation.environment import Environment
from simulation.simulator.simulator_helper import TaskPriority


class SimulateCommand(Command):

    def __init__(self, agents: Agents, environment: Environment, task_priority: TaskPriority = TaskPriority.LOW):
        self.agents: Agents = agents
        self.environment: Environment = environment
        self.task_priority: TaskPriority = task_priority

    def __eq__(self, other):
        return self.agents == other.agents

    def __hash__(self):
        return hash((self.agents))