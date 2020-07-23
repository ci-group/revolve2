from pyrevolve.evolutionary.individual import Agent
from pyrevolve.evolutionary.things.environment import Environment
from pyrevolve.evolutionary.things.performance_measures import PerformanceMeasures


class Simulator:

    def __init__(self):
        pass

    def evaluate(self, environment: Environment):
        for agent in environment.population.parents.parents:
            agent.fitness.process(self.simulate(agent))

    def simulate(self, agent: Agent) -> PerformanceMeasures:
        measures = PerformanceMeasures()
        measures.test()
        return measures
