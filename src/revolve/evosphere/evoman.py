import os
import string
import numpy as np

from nca.core.actor.fitness import Fitness
from nca.core.actor.individual import Individual

from revolve.evosphere.ecosphere import Ecosphere
from simulation.simulator.simulator_helper import SimulatorType

from third_party.evoman.demo.demo_controller import player_controller
from third_party.evoman.environment import Environment


class EvomanEcosphere(Ecosphere):

    def __init__(self, filename: string = "evoman", fitness_type: type(Fitness) = None):
        super().__init__(filename, fitness_type, SimulatorType.EVOMAN)

        if not os.path.exists(filename):
            os.makedirs(filename)

        self.filename = filename

        self.env = Environment(experiment_name=self.filename, enemies=[2], playermode="ai", level=2,
                          speed="fastest", player_controller=player_controller(10), enemy_controller="static")

    def run(self, individual: Individual):
        fitness_value, _, _, _ = self.env.play(pcont=np.array(individual.get_representation()))
        individual.fitness.add("evoman", fitness_value)

