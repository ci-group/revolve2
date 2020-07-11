from pyrevolve.evolutionary.algorithm.selection import SurvivorSelection
from pyrevolve.evolutionary.robotics import Agents


class SteadyStateSelection(SurvivorSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, agents: Agents) -> Agents:
        super().select(agents)
        new_agents = Agents()

        # TODO SS selection

        return new_agents


class AgeBasedReplacement(SurvivorSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, agents: Agents) -> Agents:
        pass


class FitnessBasedReplacement(SurvivorSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, agents: Agents) -> Agents:
        pass


class Elitism(SurvivorSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, agents: Agents) -> Agents:
        pass


class RoundRobinTournament(SurvivorSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, agents: Agents) -> Agents:
        pass


class MuLambdaSelection(SurvivorSelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, agents: Agents) -> Agents:
        pass
