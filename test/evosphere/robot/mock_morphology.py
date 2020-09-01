from revolve.robot.body.body import Body
from revolve.robot.brain.brain import Brain


class MockBody(Body):
    def develop(self):
        pass


class MockBrain(Brain):
    def develop(self):
        pass