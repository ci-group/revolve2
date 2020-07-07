import string

from pyrevolve.agent.agents import Agents
from pyrevolve.patterns.abstract.memento import Memento
from pyrevolve.population_ecology.age import Age


class Population(Memento):

    def __init__(self, id: int, agents: Agents, age: Age = Age()):
        super(Memento).__init__()
        self.id: int = id
        self.agents: Agents = agents
        self.age: Age = age

    def load(self):
        # TODO Load
        pass

    def export(self):
        #TODO export
        pass
