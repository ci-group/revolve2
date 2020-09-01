from nca.core.abstract.configurations import EvoSphereConfiguration
from nca.core.actor.actors import Actors
from nca.core.evolution.evolutionary_algorithm import EvolutionaryAlgorithm
from nca.core.evolution.evolutionary_configurations import GeneticAlgorithmConfiguration, EvolutionConfiguration

from revolve.evosphere.biosphere import Biosphere, RobotBiosphere, AgentBiosphere

from simulation.simulation_manager import SimulationManager
from simulation.simulator.simulator_command import SimulateCommand


class Evosphere:

    def __init__(self, biosphere: Biosphere,
                 evolutionary_configuration: EvolutionConfiguration = GeneticAlgorithmConfiguration(),
                 simulation: SimulationManager = SimulationManager(), debug=True):
        self.configuration = EvoSphereConfiguration()

        self.biosphere: Biosphere = biosphere
        self.evolutionary_algorithm: EvolutionaryAlgorithm = EvolutionaryAlgorithm(evolutionary_configuration)
        self.simulation: SimulationManager = simulation

        self.debug = debug
        self.terminate = False

        self.biosphere.initialize(self.configuration.number_of_agents, self.evolutionary_algorithm.initialization_type)

    def evolve(self):

        self.log("Evaluate population")

        for population in self.biosphere.populations():
            self.evaluate(population.individuals)

        # Run through iterations
        for generation_index in range(self.configuration.number_of_generations):
            self.log("Generation " + str(generation_index))
            for population in self.biosphere.populations():
                if self.evolutionary_algorithm.should_terminate(population):
                    self.terminate = True
                    break

                self.evolutionary_algorithm.run(population, self.evaluate)

            if self.terminate:
                self.log("Terminated evolution due to " + str(self.evolutionary_algorithm.termination_condition))
                break

            self.biosphere.run()

        self.log(str(self.biosphere.populations()))

    def evaluate(self, agents: Actors):
        for ecosphere in self.biosphere.ecospheres:
            self.simulation.simulate(SimulateCommand(agents, ecosphere))

    def log(self, string: str):
        if self.debug:
            print(string)


class RobotEvosphere(Evosphere):
    def __init__(self, biosphere: Biosphere = RobotBiosphere(),
                 evolutionary_configuration: EvolutionConfiguration = GeneticAlgorithmConfiguration(),
                 simulation: SimulationManager = SimulationManager(), debug=True):
        super().__init__(biosphere, evolutionary_configuration, simulation, debug)


class AgentEvosphere(Evosphere):
    def __init__(self, biosphere: Biosphere = AgentBiosphere(),
                 evolutionary_configuration: EvolutionConfiguration = GeneticAlgorithmConfiguration(),
                 simulation: SimulationManager = SimulationManager(), debug=True):
        super().__init__(biosphere, evolutionary_configuration, simulation, debug)
