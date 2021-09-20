from revolve2.nca.core.evolution.evolutionary_configurations import EvolutionaryConfiguration, GeneticAlgorithmConfiguration
from revolve2.nca.core.genome.operators.mutation_operator import SwapMutation
from revolve2.nca.core.genome.operators.recombination_operator import UniformCrossover
from revolve2.revolve.evosphere.biosphere import Biosphere
from revolve2.revolve.evosphere.evoman import EvomanEcosphere
from revolve2.revolve.evosphere.evosphere import Evosphere
from revolve2.revolve.robot.birth_clinic import IndividualBirthClinic

evolutionary_configuration: EvolutionaryConfiguration = GeneticAlgorithmConfiguration(recombination=UniformCrossover(),
                                                                                      mutation=SwapMutation())
biosphere = Biosphere(birth_clinic=IndividualBirthClinic(), ecospheres=[EvomanEcosphere()])
evolution = Evosphere(biosphere=biosphere, evolutionary_configuration=evolutionary_configuration)
evolution.evolve()
