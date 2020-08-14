from nca.core.abstract.configurations import EvoSphereConfiguration
from nca.core.agent.agents import Agents
from nca.core.ecology.population import Population
from nca.core.evolution.evolutionary_algorithm import EvolutionaryAlgorithm
from revolve.evosphere.configuration import RevolveGeneticAlgorithmConfiguration
from revolve.evosphere.world import World, GazeboWorld, Ecosphere
from simulation.simulation_manager import SimulationManager
from simulation.simulator.simulator_command import SimulateCommand


class EvoSphere:

    def __init__(self, biosphere: Biosphere,
                 evolutionary_algorithm: EvolutionaryAlgorithm,
                 simulation: SimulationManager):
        self.configuration = EvoSphereConfiguration()

        self.biosphere: Biosphere = biosphere

        self.evolutionary_algorithm: EvolutionaryAlgorithm = evolutionary_algorithm

        self.simulation = simulation

    def evolve(self):
        # load and initialize
        self.biosphere.initialize(self.configuration.number_of_agents)
        self.evolutionary_algorithm.initialize(self.biosphere.populations())

        print("Evaluate population")
        for population in self.biosphere.populations():
            self.evaluate(population.individuals)

        # Run through iterations
        for generation_index in range(self.configuration.number_of_generations):
            print("Generation ", generation_index)

            for population in self.biosphere.populations():
                if self.evolutionary_algorithm.should_terminate(population):
                    print("Terminated evolution due to termination condition.")
                    break

                self.evolutionary_algorithm.run(population, self.evaluate)

            self.biosphere.run()

    def evaluate(self, agents: Agents):
        return self.simulation.simulate(SimulateCommand(agents, self.biosphere))


class DefaultEvoSphere(EvoSphere):

    def __init__(self, world: World = GazeboWorld(),
                 evolutionary_algorithm: EvolutionaryAlgorithm = EvolutionaryAlgorithm(RevolveGeneticAlgorithmConfiguration()),
                 simulation: SimulationManager = SimulationManager()):
        super().__init__(world, evolutionary_algorithm, simulation)

