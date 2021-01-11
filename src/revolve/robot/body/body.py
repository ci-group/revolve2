from abc import ABC

from revolve.robot.body.body_blueprint import BodyBlueprint
from revolve.robot.body.robogen.robogen_body_measures import MorphologicalMeasures


class Body(ABC):

    def __init__(self):
        self.sensors = []
        self.actuators = []

    def get_sensors(self):
        return self.sensors

    def get_actuators(self):
        return self.actuators

    def get_blueprint(self):
        return BodyBlueprint(len(self.sensors), len(self.actuators))


class BodyDevelopmentalMeasures:
    pass


class RobotBody(Body):

    def __init__(self):
        super().__init__()

        self.morphological_measures: MorphologicalMeasures = None
        self.developmental_measures: BodyDevelopmentalMeasures = None


class AgentBody(Body):

    pass
