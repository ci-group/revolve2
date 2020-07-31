from typing import List

from pyrevolve.evolutionary.algorithm.selection.parent_selection import TournamentSelection
from pyrevolve.evolutionary.ecology.population_management import PopulationManagement
from pyrevolve.evolutionary.things.environment import Environment
from pyrevolve.evolutionary.things.evosphere import EvoSphere
from pyrevolve.experiment.experiment_manager import ExperimentManager
from pyrevolve.evolutionary.ecology import PopulationEcology

experiment_manager = ExperimentManager()


def run():

    population_ecology = PopulationEcology(PopulationManagement(TournamentSelection()))
    environments: List[Environment] = [Environment("nursery")]#, Environment("arena")]

    evosphere = EvoSphere(population_ecology, environments)

    evosphere.run()


if __name__ == "__main__":
    run()