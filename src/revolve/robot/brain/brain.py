from abc import ABC


class Brain(ABC):
    pass


class BrainDevelopmentalMeasures:
    pass


class RobotBrain(Brain):

    def __init__(self):
        self.developmental_measures: BrainDevelopmentalMeasures = None


class AgentBrain(Brain):
    pass
