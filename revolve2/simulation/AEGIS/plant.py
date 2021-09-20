import math

import numpy as np

from revolve2.simulation.AEGIS.spatial_entity import SpatialEntity, Coordinate


class Plant(SpatialEntity):

    max_energy = 75

    min_birth_energy = 0.5
    max_birth_energy = 1.5

    living_threshold = 0.5
    plant_growth = 1.4
    eat_factor = 0.5

    multiplication_threshold = 25.0
    launch_threshold = 50.0

    seed_spread_consumption = 0.25
    seed_launch_consumption = 0.05

    def __init__(self, coordinate: Coordinate):
        super().__init__(coordinate, birth_energy=np.random.uniform(self.min_birth_energy, self.max_birth_energy))
        #self.eaten: bool = False  # check if eaten in the previous cycle, then do not grow this cycle due to transformation shock.

    # Each timestep the plant grows with a standard
    def step(self) -> bool:  # test if left alive.
        if self.energy > self.living_threshold:
            self.energy = min(self.max_energy, self.plant_growth * self.energy)
            return True
        return False

    def spread_seed(self):
        return self.energy >= self.multiplication_threshold

    def spread(self):
        self.energy = self.seed_spread_consumption * self.energy

    def launch_seed(self):
        return self.energy >= self.launch_threshold

    def launched(self):
        self.energy = self.seed_launch_consumption * self.energy

    # Animal eats plant, plants is left with a percentage of the original energy, other other part is
    def consume(self) -> float:
        plant_remaining_energy = self.eat_factor * self.energy
        animal_consumed_energy = (1 - self.eat_factor) * self.energy
        self.energy = plant_remaining_energy
        return animal_consumed_energy

    def render(self):
        return " P%03d " % math.ceil(self.energy)

    def alive(self):
        return self.energy > self.living_threshold
