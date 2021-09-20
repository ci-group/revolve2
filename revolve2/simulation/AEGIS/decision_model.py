import numpy as np

from revolve2.nca.core.ecology.population import Population
from revolve2.simulation.AEGIS.animal import Chromosome, Animal
from revolve2.simulation.AEGIS.scape_environment import Action


def decide_all(population: Population):
    return [decision_model(agent) for agent in population.individuals]


def decision_model(animal: Animal):
    representation = animal.get_representation()
    actions = [Chromosome.Strength, Chromosome.Energy, Chromosome.Mating]
    probabilities = [np.random.random() * representation[action] for action in actions]
    action_index = np.argmax(probabilities)
    return Action(action_index)


if __name__ == "__main__":
    representation = [0.5, 0.2, 0.5, 0.5, 0.5]
    actions = [Chromosome.Strength, Chromosome.Energy, Chromosome.Mating]
    for i in range(10):
        probabilities = [np.random.random() * representation[action] for action in actions]
        action_index = np.argmax(probabilities)
        print(Action(action_index))
