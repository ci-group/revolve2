from pyrevolve.evolutionary.robotics import Agent
from pyrevolve.evolutionary.things.environment import Environment
from pyrevolve.evolutionary.things import PerformanceMeasures


class Simulator:

    def __init__(self):
        pass

    def evaluate(self, environment: Environment):
        for agent in environment.population.agents.agents:
            agent.fitness.process(self.simulate(agent))

    def simulate(self, agent: Agent) -> PerformanceMeasures:
        measures = PerformanceMeasures()
        measures.test()
        return measures
