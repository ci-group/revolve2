import math

from nca.core.agent.age import Age
from nca.core.abstract.sequential_identifier import AgentIdentifier
from nca.core.genome.representation import Representation
from revolve.robot.brain.brain import Brain
from simulation.simulation_measures import SimulationMeasures


class Individual:
    identifier = AgentIdentifier()

    def __init__(self, representation: Representation):
        super().__init__()
        self.id: int = self.identifier.id()
        self.age: Age = Age()

        self.representation: Representation = representation
        self.measures = None
        self.fitness: float = -math.inf

    def __lt__(self, other):
        return self.id < other.id

    def performance(self, measures: SimulationMeasures, fitness_function):
        self.measures = measures
        self.fitness = fitness_function(self)

    def __repr__(self):
        return str(self.id) + " " + str(self.representation) + " " + str(self.fitness) + "\n"


class Agent(Individual):
    def __init__(self, brain: Brain):
        super().__init__(brain)
