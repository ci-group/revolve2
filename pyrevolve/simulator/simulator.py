from pyrevolve.evolutionary import Individual
from pyrevolve.evolutionary.things.environment import Environment
from pyrevolve.evolutionary.things.performance_measures import PerformanceMeasures


class Simulator:

    def __init__(self):
        pass

    def evaluate(self, environment: Environment):
        # todo fix
        for agents in environment.agents_list:
            for individual in agents:
                individual.fitness.process(self.simulate(individual))

    def simulate(self, individual: Individual) -> PerformanceMeasures:
        measures = PerformanceMeasures()
        measures.test()
        return measures
