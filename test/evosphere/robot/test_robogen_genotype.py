import unittest

from revolve2.nca.core.genome.operators.mutation_operator import InsertMutation
from revolve2.revolve.robot.body.robogen.robogen_genotype import SelfOrganizingRobogenGenotype


class SelfOrganizingRobogenGenotypeTest(unittest.TestCase):

    def test_same(self):

        genotype = SelfOrganizingRobogenGenotype()
        # encoding = genotype()
        # print(encoding)
        mutation = InsertMutation()
        mutation._mutate(genotype['symbols'])
