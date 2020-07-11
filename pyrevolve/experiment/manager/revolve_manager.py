from typing import List

from pyrevolve.evolutionary.things.environment import Environment
from pyrevolve.evolutionary.things.evosphere import EvoSphere
from pyrevolve.experiment.experiment_manager import ExperimentManager
from pyrevolve.ecology import PopulationEcology
from pyrevolve.ecology import PopulationManagement
from pyrevolve.evolutionary.algorithm import TournamentSelection

experiment_manager = ExperimentManager()


def run():

    population_ecology = PopulationEcology(PopulationManagement(TournamentSelection()))
    environments: List[Environment] = [Environment("nursery")]#, Environment("arena")]

    evosphere = EvoSphere(population_ecology, environments)

    evosphere.run()


if __name__ == "__main__":
    run()