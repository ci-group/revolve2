import math

import numpy as np

from revolve2.nca.core.actor.individual import Individual
from revolve2.nca.core.genome.genotype import Genotype
from revolve2.simulation.AEGIS.spatial_entity import Coordinate, Direction


class Chromosome:
    Muscle = 0
    Skin = 1
    Strength = 2
    Energy = 3
    Mating = 4


class Animal(Individual):

    max_energy = 150

    min_death_age = 25
    max_death_age = 50

    action_consumption = []

    attack_energy = 0
    eat_energy = 0
    move_energy = 1
    mating_energy = 10

    mating_energy_threshold = 10

    child_bearing_age = 15
    mating_recovery_duration = 5

    vision = 3

    def __init__(self, genotype: Genotype):
        super().__init__(genotype)
        # randomly assign either male (0) of female (1)
        self.sex = np.random.randint(1)
        self.genotype: Genotype = genotype

        self.coordinate: Coordinate = None
        self.age = 0
        self.death_age = np.random.uniform(self.min_death_age, self.max_death_age)
        self.energy: float = 0

        self.sex_recovery: int = 0

    def initialize(self, coordinate: Coordinate, birth_energy: int = 50):
        self.coordinate: Coordinate = coordinate
        self.energy = birth_energy

    def speed(self):
        return math.ceil(self.energy / 10)

    def life(self):
        return self.energy

    def alive(self):
        self.age = min(self.death_age, self.age + 1)
        self.sex_recovery = max(0, self.sex_recovery - 1)
        return self.energy > 0.0 or self.age >= self.death_age

    def move(self, direction: Direction):
        self.coordinate += direction.value
        self.energy -= self.move_energy

    def eat(self, energy):
        self.energy += energy
        self.energy -= self.eat_energy

    def distance(self, other_animal):
        return self.coordinate - other_animal.coordinate

    def attack(self, other):
        if self.energy > other.energy:
            other.take_damage()
        elif self.energy < other.energy:
            self.take_damage()
        else:
            self.take_damage()
            other.take_damage()

    def take_damage(self):
        self.energy -= self.attack_energy

    def is_mate(self, other_animal) -> bool:
        return self.energy > self.mating_energy and other_animal.energy > self.mating_energy
               #self.age >= self.child_bearing_age and other_animal.age >= self.child_bearing_age and \
               #self.sex != other_animal.sex and self.sex_recovery == 0 and self.sex_recovery == 0

    def act(self):
        pass

    def mating(self):
        self.sex_recovery = self.mating_recovery_duration
        self.energy -= self.mating_energy
