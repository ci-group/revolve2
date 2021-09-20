import numpy as np

from revolve2.nca.core.actor.actors import Actors
from revolve2.nca.core.evolution.selection.selection import MortalitySelection


class NullMortalitySelection(MortalitySelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, individuals: Actors, offspring: Actors):
        individuals.extend(offspring)
        return individuals


class OffspringMortalitySelection(MortalitySelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, individuals: Actors, offspring: Actors):
        amount_of_offspring = len(offspring)
        offspring_samples = np.random.sample((amount_of_offspring,))

        for index, actor in enumerate(offspring):
            if offspring_samples[index] < self.mortality_percentage:
                offspring.remove(actor)

        individuals.extend(offspring)
        return individuals


class PopulationMortalitySelection(MortalitySelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, individuals: Actors, offspring: Actors):
        offspring_samples = np.random.sample((len(individuals),))

        for index, actor in enumerate(individuals):
            if len(individuals) + len(offspring) == self.configuration.population_size:
                break

            if offspring_samples[index] < self.mortality_percentage:
                individuals.remove(actor)

        individuals.extend(offspring)
        return individuals


class ConstantMortalitySelection(MortalitySelection):

    def __init__(self):
        super().__init__()

    def algorithm(self, individuals: Actors, offspring: Actors):
        individuals.extend(offspring)
        amount_of_actors = len(individuals)
        offspring_samples = np.random.sample((amount_of_actors,))

        for index, actor in enumerate(individuals):
            if len(individuals) == self.configuration.population_size:
                break

            if offspring_samples[index] < self.mortality_percentage:
                individuals.remove(actor)

        return individuals
