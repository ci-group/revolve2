from typing import Dict

from abstract.configurations import EvoSphereConfiguration
from nca.core.actor.actors import Actors
from nca.core.evolution.evolutionary_algorithm import EvolutionaryAlgorithm
from nca.core.evolution.evolutionary_configurations import GeneticAlgorithmConfiguration, EvolutionaryConfiguration

from revolve.evosphere.biosphere import Biosphere, RobotBiosphere, IndividualBiosphere

from simulation.simulation_manager import SimulationManager
from simulation.simulation_measures import SimulationMeasures
from simulation.simulator.simulator_command import SimulationRequest


class Evosphere:

    def __init__(self, biosphere: Biosphere,
                 evolutionary_configuration: EvolutionaryConfiguration = GeneticAlgorithmConfiguration(),
                 simulation: SimulationManager = SimulationManager(), debug=True):
        self.configuration = EvoSphereConfiguration()

        self.biosphere: Biosphere = biosphere
        self.evolutionary_algorithm: EvolutionaryAlgorithm = EvolutionaryAlgorithm(evolutionary_configuration)
        self.simulation: SimulationManager = simulation

        self.debug = debug
        self.terminate = False

        self.biosphere.initialize(self.configuration.number_of_agents, self.evolutionary_algorithm.initialization_type)

    def evolve(self):

        for population in self.biosphere.populations():
            self.log("Evaluate population " + str(len(population.individuals)))
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
        return self.biosphere.population_ecology

    def evaluate(self, agents: Actors):
        for ecosphere in self.biosphere.ecospheres:
            results: Dict[int, SimulationMeasures] = self.simulation.simulate(SimulationRequest(agents, ecosphere, self.biosphere.birth_clinic))
            for agent in agents:
                agent.fitness.add(results[agent.id].fitness)
                print(agent.fitness.value(), agent.fitness.objectives)

    def log(self, string: str):
        if self.debug:
            print(string)


class RobotEvosphere(Evosphere):
    def __init__(self, biosphere: Biosphere = RobotBiosphere(),
                 evolutionary_configuration: EvolutionaryConfiguration = GeneticAlgorithmConfiguration(),
                 simulation: SimulationManager = SimulationManager(), debug=True):
        super().__init__(biosphere, evolutionary_configuration, simulation, debug)


class AgentEvosphere(Evosphere):
    def __init__(self, biosphere: Biosphere = IndividualBiosphere(),
                 evolutionary_configuration: EvolutionaryConfiguration = GeneticAlgorithmConfiguration(),
                 simulation: SimulationManager = SimulationManager(), debug=True):
        super().__init__(biosphere, evolutionary_configuration, simulation, debug)
