from pyrevolve.evolutionary.robotics import Robot
from pyrevolve.evolutionary.robotics import Robots
from pyrevolve.shared.configurations import RobotConfiguration
from pyrevolve.shared.sequential_identifier import SequentialIdentifier


class BirthClinic:

    def __init__(self):
        self.robot_identifier = SequentialIdentifier()
        self.configuration = RobotConfiguration()

    def create_robots(self) -> Robots:
        robots: Robots = Robots()

        for robot_index in range(self.configuration.number_of_robots):
            robots.add(self.build_robot())

        return robots

    def build_robot(self) -> Robot:
        return Robot(self.robot_identifier.increment(), self.configuration.fitness)
