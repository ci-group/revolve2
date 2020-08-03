from typing import List

from nca.core.evolution.selection.parent_selection import TournamentSelection
from nca.core.ecology.population_management import PopulationManagement
from nca.revolve.evosphere.environment import Environment
from nca.revolve import EvoSphere
from nca.experiment.experiment_manager import ExperimentManager
from nca.core.ecology import PopulationEcology

experiment_manager = ExperimentManager()


def run():

    population_ecology = PopulationEcology(PopulationManagement(TournamentSelection()))
    environments: List[Environment] = [Environment("nursery")]#, Environment("arena")]

    evosphere = EvoSphere(population_ecology, environments)

    evosphere.run()


if __name__ == "__main__":
    run()