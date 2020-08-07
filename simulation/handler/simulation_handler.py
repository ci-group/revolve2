from abc import abstractmethod

from revolve.robot.robot import Robot


class Handler:
    pass


class SimulationHandler(Handler):

    def __init__(self, follow_up: Handler):
        self.follow_up = follow_up

    @abstractmethod
    def handle(self, robot: Robot):
        pass
