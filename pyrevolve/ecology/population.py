from pyrevolve.ecology import Age
from pyrevolve.evolutionary.robotics import Robot
from pyrevolve.evolutionary.robotics import Robots


class Compatibility(object):

    def __call__(self, *args, **kwargs):
        # robots =
        # robot =
        pass


class Population:

    compatibility: Compatibility = Compatibility()

    def __init__(self, population_id: int, robots: Robots):
        # Basics
        self.id: int = population_id
        self.robots: Robots = robots
        self.age: Age = Age()

        # Speciation
        self.representative = None

    def __get__(self):
        return self.robots

    def compatible(self, robot: Robot):
        if self.compatibility(self.robots, robot):
            self.robots.add(robot)
            return True

        return False
