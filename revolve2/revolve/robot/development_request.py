from revolve2.nca.core.genome.genotype import Genotype
from revolve2.revolve.evosphere.ecosphere import Ecosphere
from revolve2.revolve.robot.body.body import BodyBlueprint


class DevelopmentRequest:

    def __init__(self, individual_identifier: int, genotype: Genotype, ecosphere: Ecosphere):
        self.individual_identifier: int = individual_identifier
        self.genotype: Genotype = genotype
        self.ecosphere: Ecosphere = ecosphere


class BodyDevelopmentRequest:

    def __init__(self, individual_identifier: int, genotype: Genotype, ecosphere: Ecosphere):
        self.individual_identifier: int = individual_identifier
        self.genotype: Genotype = genotype
        self.ecosphere: Ecosphere = ecosphere


class BrainDevelopmentRequest:

    def __init__(self, individual_identifier: int, genotype: Genotype, ecosphere: Ecosphere, body_blueprint: BodyBlueprint):
        self.individual_identifier: int = individual_identifier
        self.genotype: Genotype = genotype
        self.ecosphere: Ecosphere = ecosphere
        self.body_blueprint: BodyBlueprint = body_blueprint
