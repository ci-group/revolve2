from nca.core.genome.genotype import Genotype
from revolve.evosphere.ecosphere import Ecosphere
from revolve.robot.body.body import BodyBlueprint


class DevelopmentRequest:

    def __init__(self, genotype: Genotype, ecosphere: Ecosphere):
        self.genotype: Genotype = genotype
        self.ecosphere: Ecosphere = ecosphere


class BodyDevelopmentRequest:

    def __init__(self, genotype: Genotype, ecosphere: Ecosphere):
        self.genotype: Genotype = genotype
        self.ecosphere: Ecosphere = ecosphere


class BrainDevelopmentRequest:

    def __init__(self, genotype: Genotype, ecosphere: Ecosphere, body_blueprint: BodyBlueprint):
        self.genotype: Genotype = genotype
        self.ecosphere: Ecosphere = ecosphere
        self.body_blueprint: BodyBlueprint = body_blueprint
