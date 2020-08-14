
from nca.core.agent.individual import Individual
from revolve.robot.body.body import Body
from revolve.robot.brain.brain import Brain
from revolve.robot.morphology import MorphologyType


class Robot(Individual):

    def __init__(self, body: Body = None, brain: Brain = None):
        super().__init__({MorphologyType.BRAIN: brain, MorphologyType.BODY: body})
