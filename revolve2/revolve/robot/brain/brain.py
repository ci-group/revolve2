from abc import ABC


class Brain(ABC):
    pass


class BrainDevelopmentalMeasures:
    pass


class RobotBrain(Brain, ABC):

    def __init__(self):
        self.developmental_measures: BrainDevelopmentalMeasures = None


class AgentBrain(Brain, ABC):
    pass
