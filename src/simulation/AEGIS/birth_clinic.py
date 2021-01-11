from nca.core.actor.individual import Individual
from revolve.robot.birth_clinic import BirthClinic
from simulation.AEGIS.animal import Animal
from simulation.AEGIS.spatial_entity import Coordinate


class AnimalBirthClinic(BirthClinic):

    def __init__(self):
        super().__init__()

    def _create(self, individual: Individual, ecosphere) -> Animal:
        animal = Animal(individual.genotype)
        animal.initialize(Coordinate.random_coordinate(ecosphere.height, ecosphere.width))
        return animal
