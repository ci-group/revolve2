from nca.core.actor.individual import Individual
from revolve.robot.robot import Robot
from visualization.visualization import StatisticsVisualization, Visualization


class IndividualVisualization(StatisticsVisualization):

    def __init__(self, individual: Individual):
        super().__init__()
        self.individual = individual


class RobotVisualization(IndividualVisualization):
    def __init__(self, robot: Robot):
        super().__init__()
        self.individual = robot


class BrainVisualization(Visualization, RobotVisualization):
    pass


class BodyVisualization(Visualization, RobotVisualization):
    pass


class TrajectoryVisualization(Visualization, IndividualVisualization):
    pass
