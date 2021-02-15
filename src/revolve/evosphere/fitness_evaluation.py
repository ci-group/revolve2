import math
import numpy as np

from nca.core.actor.fitness_evaluation import FitnessEvaluation, FitnessEvaluations
from revolve.robot.robot import Robot


class DisplacementFitnessEvaluation(FitnessEvaluation):

    def __call__(self, robot: Robot):
        delta_x = robot.measures['displacement_x'][-1] - robot.measures['displacement_x'][0]
        delta_y = robot.measures['displacement_y'][-1] - robot.measures['displacement_y'][0]
        return math.sqrt(delta_x**2 + delta_y**2)


class RotationalFitnessEvaluation(FitnessEvaluation):

    def __call__(self, robot: Robot):
        return np.sum(robot.measures['rotation'])


class DisplacementRotationalFitnessEvaluations(FitnessEvaluations):

    def __init__(self):
        super().__init__([DisplacementFitnessEvaluation(), RotationalFitnessEvaluation()])
