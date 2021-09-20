import copy
from enum import Enum
from typing import Dict, List

import gym

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm

from revolve2.nca.core.ecology.population import Population
from revolve2.nca.core.genome.operators.mutation_operator import ReplaceMutation
from revolve2.simulation.AEGIS.animal import Animal
from revolve2.simulation.AEGIS.birth_clinic import AnimalBirthClinic
from revolve2.simulation.AEGIS.plant import Plant
from revolve2.simulation.AEGIS.spatial_entity import Coordinate, Direction


# https://github.com/koulanurag/ma-gym/tree/master/ma_gym/envs/utils
class MultiAgentActionSpace(list):
    def __init__(self, agents_action_space):
        for x in agents_action_space:
            assert isinstance(x, gym.spaces.space.Space)

        super(MultiAgentActionSpace, self).__init__(agents_action_space)
        self._agents_action_space = agents_action_space

    def sample(self):
        """ samples action for each agent from uniform distribution"""
        return [agent_action_space.sample() for agent_action_space in self._agents_action_space]


# https://github.com/koulanurag/ma-gym/tree/master/ma_gym/envs/utils
class MultiAgentObservationSpace(list):
    def __init__(self, agents_observation_space):
        for x in agents_observation_space:
            assert isinstance(x, gym.spaces.space.Space)

        super().__init__(agents_observation_space)
        self._agents_observation_space = agents_observation_space

    def sample(self):
        """ samples observations for each agent from uniform distribution"""
        return [agent_observation_space.sample() for agent_observation_space in self._agents_observation_space]

    def contains(self, obs):
        """ contains observation """
        for space, ob in zip(self._agents_observation_space, obs):
            if not space.contains(ob):
                return False
        else:
            return True


class Action(Enum):
    Attack = 0
    Eat = 1
    Mate = 2
    Move = 3


class ScapeEnvironment(gym.Env):

    population_limit = 5000

    def __init__(self, height: int, width: int, initial_flora_count: int, population: Population):
        self.height = height
        self.width = width

        self.initial_flora_count: int = initial_flora_count
        self.flora_world: Dict[Coordinate, Plant] = {}

        self.birth_clinic = AnimalBirthClinic()

        animals = self.birth_clinic.create(population.individuals, self)

        self.population: Population = Population(animals)
        self.animal_world: Dict[Coordinate, List[Animal]] = {}

        self.mutation = ReplaceMutation()

        self.history = dict(
            flora_count=[],
            animal_count=[],
            flora_energy=[],
            animal_energy=[]
        )

        self.timestep: int = 0
        self.reset()

    def reset(self):
        self.timestep = 0

        self.flora_world.clear()

        for plant_index in range(self.initial_flora_count):
            coordinate: Coordinate = self._random_unique_coordinate()
            self.flora_world[coordinate] = Plant(coordinate)

        self.animal_world: Dict[Coordinate, List[Animal]] = {}

        # Fill dictionary reference linking coordinates and animals.
        for animal in self.population.individuals:
            coordinate = animal.coordinate
            if coordinate in self.animal_world:
                self.animal_world[coordinate].append(animal)
            else:
                self.animal_world[coordinate] = [animal]

    def step(self, actions: List):
        print("Time: ", self.timestep)
        print("animals ", len(self.population.individuals))
        print("flora ", len(self.flora_world.keys()))
        # Develop Flora using their own developmental stages and reproduction rates.
        coordinate_keys = list(self.flora_world.keys())
        for coordinate_key in coordinate_keys:
            self.flora_world[coordinate_key].step()

            plant: Plant = self.flora_world[coordinate_key]

            if not plant.alive():
                del self.flora_world[coordinate_key]
                continue


            # Check if the plant can spread it seed, if so then choose a random neighboring coordinate to reproduce to.
            if plant.spread_seed():
                coordinates = plant.coordinate.neighbours()
                seed_coordinate = np.random.choice(coordinates)
                added = self._add_plant(seed_coordinate, Plant(seed_coordinate))
                if added:
                    plant.spread()

            # Check if the plant can launch it seed, if so then choose a random coordinate to reproduce to.
            if plant.launch_seed():
                seed_coordinate = Coordinate.random_coordinate(self.height, self.width)
                added = self._add_plant(seed_coordinate, Plant(seed_coordinate))
                if added:
                    plant.launched()

        new_animals = []
        died_animals = []
        # Perform the actions for all agents.
        for agent_index, action in enumerate(actions):
            animal: Animal = self.population.individuals[agent_index]

            if not animal.alive():
                died_animals.append(animal)
                # TODO mortality detection outside the loop. Maybe through done flag.
                continue

            # Mate with animal of opposite sex if it is in the same place
            if action == Action.Mate:
                mated = False
                #for other_animal in self.animal_world[animal.coordinate]:
                other_animal = np.random.choice(self.population.individuals)
                if animal is not other_animal and animal.distance(other_animal) < self.width / 2:
                    if animal.is_mate(other_animal):
                        animal.mating()
                        other_animal.mating()

                        # TODO do mutation and recombination flags for mating.
                        new_animals.append(copy.deepcopy(animal))
                        mated = True
                        #break

                if not mated:
                    action = Action.Move

            # Attack another animal if it is in the same place
            if action == Action.Attack:
                attacked = False
                if animal.coordinate in self.animal_world:
                    for other_animal in self.animal_world[animal.coordinate]:
                        if animal is not other_animal:
                            animal.attack(other_animal)
                            attacked = True
                            break
                if not attacked:
                    action = Action.Move

            # Eat plant if there is any left.
            if action == Action.Eat:
                eaten = False
                if animal.coordinate in self.flora_world:
                    plant = self.flora_world[animal.coordinate]
                    animal.eat(plant.consume())
                    eaten = True

                if not eaten:
                    action = Action.Move

            # Move to a valid random direction
            if action == Action.Move:
                previous_coordinate = copy.deepcopy(animal.coordinate)
                speed: int = animal.speed()
                for step in range(speed):
                    animal.move(self._agent_direction(animal))

                self.animal_world[previous_coordinate].remove(animal)
                if animal.coordinate in self.animal_world:
                    self.animal_world[animal.coordinate].append(animal)
                else:
                    self.animal_world[animal.coordinate] = [animal]

        if len(new_animals) > 0:
            print("animals born ", len(new_animals))
            birthed_animals = self.birth_clinic.create(new_animals, self)
            for animal in birthed_animals:
                if len(self.population.individuals) == self.population_limit:
                    break
                self.population.individuals.append(animal)
                if animal.coordinate in self.animal_world:
                    self.animal_world[animal.coordinate].append(animal)
                else:
                    self.animal_world[animal.coordinate] = [animal]

        if len(died_animals) > 0:
            print("animals died ", len(died_animals))
            for animal in died_animals:
                self.population.individuals.remove(animal)
                self.animal_world[animal.coordinate].remove(animal)

        self.history['flora_count'].append(len(self.flora_world.keys()))
        self.history['animal_count'].append(len(self.population))
        self.history['flora_energy'].append(np.mean([flora.energy for flora in self.flora_world.values()]))
        self.history['animal_energy'].append(np.mean([animal.energy for animal in self.population.individuals]))

        for individual in self.population.individuals:
            individual.genotype = self.mutation(individual.genotype)

        self.timestep += 1

    def render(self, mode='human'):
        colors = np.zeros([self.height, self.width, 3])

        for coordinate in self.animal_world.keys():
            for animal in self.animal_world[coordinate]:
                colors[coordinate.x, coordinate.y, 0] = animal.life()

        for coordinate in self.flora_world.keys():
            colors[coordinate.x, coordinate.y, 1] = self.flora_world[coordinate].life()

        # Normalize visualization between 0.0 and 1.0
        max_value_animalia = np.max(colors[:, :, 0])
        max_value_flora = np.max(colors[:, :, 1])
        colors[:, :, 0] /= max_value_animalia
        colors[:, :, 1] /= max_value_flora

        filename = "../../../log/aegis/iteration_" + str(self.timestep) + ".png"
        plt.figure(figsize=[4, 4])
        plt.imshow(colors, interpolation='nearest', cmap=cm.gist_rainbow, aspect='auto')
        plt.axis('off')
        plt.savefig(filename)
        plt.close()
        return filename

    def _add_plant(self, coordinate: Coordinate, plant: Plant):
        if coordinate.check(self.height, self.width):
            if coordinate not in self.flora_world.keys():
                self.flora_world[coordinate] = plant
                return True

        return False

    def _random_unique_coordinate(self):
        coordinate: Coordinate = Coordinate.random_coordinate(self.height, self.width)
        while coordinate in self.flora_world.keys():
            coordinate = Coordinate.random_coordinate(self.height, self.width)

        return coordinate

    def _random_direction(self, coordinate):
        directions = []
        for direction in Direction:
            next_coordinate = coordinate + direction.value
            if next_coordinate.check(self.height, self.width):
                directions.append(direction)
        return np.random.choice(directions)

    def _agent_direction(self, animal: Animal):
        for step in range(animal.vision):
            for direction in Direction:
                new_coordinate = copy.deepcopy(animal.coordinate) + direction.value * (step+1)
                # do not have to check if it is valid, because there are only valid coordinates in flora world.
                if new_coordinate in self.flora_world:
                    return direction

        return np.random.choice(Direction)
