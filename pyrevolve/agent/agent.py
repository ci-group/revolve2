from abc import ABC
from typing import List

from pyrevolve.agent.fitness import Fitness


class Agent(ABC):

    def __init__(self, id, fitness: Fitness = Fitness(), parents: List[__class__] = None):
        super().__init__()
        self.id = id
        self.fitness: Fitness = fitness
        self.parents: List[Agent] = parents


