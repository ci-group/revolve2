"""Main script for the example."""

import logging
import pickle
from typing import Any

import config
import multineat
import numpy as np
import numpy.typing as npt
from evaluator import Evaluator
from genotype import Genotype
from individual import Individual
from revolve2.experimentation.evolution import ModularRobotEvolution
from revolve2.experimentation.evolution.abstract_elements import Reproducer, Selector
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.optimization.ea import population_management, selection
from revolve2.experimentation.rng import make_rng_time_seed


class ParentSelector(Selector):
    """Selector class for parent selection."""

    rng: np.random.Generator
    offspring_size: int

    def __init__(self, offspring_size: int, rng: np.random.Generator) -> None:
        """
        Initialize the parent selector.

        :param offspring_size: The offspring size.
        :param rng: The rng generator.
        """
        self.offspring_size = offspring_size
        self.rng = rng

    def select(
        self, population: list[Individual], **kwargs: Any
    ) -> tuple[npt.NDArray[np.int_], dict[str, list[Individual]]]:
        """
        Select the parents.

        :param population: The population of robots.
        :param kwargs: Other parameters.
        :return: The parent pairs.
        """
        return np.array(
            [
                selection.multiple_unique(
                    selection_size=2,
                    population=[individual.genotype for individual in population],
                    fitnesses=[individual.fitness for individual in population],
                    selection_function=lambda _, fitnesses: selection.tournament(
                        rng=self.rng, fitnesses=fitnesses, k=1
                    ),
                )
                for _ in range(self.offspring_size)
            ],
        ), {"parent_population": population}


class SurvivorSelector(Selector):
    """Selector class for survivor selection."""

    rng: np.random.Generator

    def __init__(self, rng: np.random.Generator) -> None:
        """
        Initialize the parent selector.

        :param rng: The rng generator.
        """
        self.rng = rng

    def select(
        self, population: list[Individual], **kwargs: Any
    ) -> tuple[list[Individual], dict[str, Any]]:
        """
        Select survivors using a tournament.

        :param population: The population the parents come from.
        :param kwargs: The offspring, with key 'offspring_population'.
        :returns: A newly created population.
        :raises ValueError: If the population is empty.
        """
        offspring = kwargs.get("children")
        offspring_fitness = kwargs.get("child_task_performance")
        if offspring is None or offspring_fitness is None:
            raise ValueError(
                "No offspring was passed with positional argument 'children' and / or 'child_task_performance'."
            )

        original_survivors, offspring_survivors = population_management.steady_state(
            old_genotypes=[i.genotype for i in population],
            old_fitnesses=[i.fitness for i in population],
            new_genotypes=offspring,
            new_fitnesses=offspring_fitness,
            selection_function=lambda n, genotypes, fitnesses: selection.multiple_unique(
                selection_size=n,
                population=genotypes,
                fitnesses=fitnesses,
                selection_function=lambda _, fitnesses: selection.tournament(
                    rng=self.rng, fitnesses=fitnesses, k=2
                ),
            ),
        )

        return [
            Individual(
                population[i].genotype,
                population[i].fitness,
            )
            for i in original_survivors
        ] + [
            Individual(
                offspring[i],
                offspring_fitness[i],
            )
            for i in offspring_survivors
        ], {}


class CrossoverReproducer(Reproducer):
    """A simple crossover reproducer using multineat."""

    rng: np.random.Generator
    innov_db_body: multineat.InnovationDatabase
    innov_db_brain: multineat.InnovationDatabase

    def __init__(
        self,
        rng: np.random.Generator,
        innov_db_body: multineat.InnovationDatabase,
        innov_db_brain: multineat.InnovationDatabase,
    ):
        """
        Initialize the reproducer.

        :param rng: The ranfom generator.
        :param innov_db_body: The innovation database for the body.
        :param innov_db_brain: The innovation database for the brain.
        """
        self.rng = rng
        self.innov_db_body = innov_db_body
        self.innov_db_brain = innov_db_brain

    def reproduce(
        self, population: npt.NDArray[np.int_], **kwargs: Any
    ) -> list[Genotype]:
        """
        Reproduce the population by crossover.

        :param population: The parent pairs.
        :param kwargs: Additional keyword arguments.
        :return: The genotypes of the children.
        :raises ValueError: If the parent population is not passed as a kwarg `parent_population`.
        """
        parent_population: list[Individual] | None = kwargs.get("parent_population")
        if parent_population is None:
            raise ValueError("No parent population given.")

        offspring_genotypes = [
            Genotype.crossover(
                parent_population[parent1_i].genotype,
                parent_population[parent2_i].genotype,
                self.rng,
            ).mutate(self.innov_db_body, self.innov_db_brain, self.rng)
            for parent1_i, parent2_i in population
        ]
        return offspring_genotypes


def find_best_robot(
    current_best: Individual | None, population: list[Individual]
) -> Individual:
    """
    Return the best robot between the population and the current best individual.

    :param current_best: The current best individual.
    :param population: The population.
    :returns: The best individual.
    """
    return max(
        population if current_best is None else [current_best] + population,
        key=lambda x: x.fitness,
    )


def main() -> None:
    """Run the program."""
    # Set up logging.
    setup_logging(file_name="log.txt")

    # Set up the random number generator.
    rng = make_rng_time_seed()

    # CPPN innovation databases.
    # If you don't understand CPPN, just know that a single database is shared in the whole evolutionary process.
    # One for body, and one for brain.
    innov_db_body = multineat.InnovationDatabase()
    innov_db_brain = multineat.InnovationDatabase()

    """
    Here we initialize the components used for the evolutionary process.

    - evaluator: Allows us to evaluate a population of modular robots.
    - parent_selector: Allows us to select parents from a population of modular robots.
    - survivor_selector: Allows us to select survivors from a population.
    - crossover_reproducer: Allows us to generate offspring from parents.
    - modular_robot_evolution: The evolutionary process as a object that can be iterated.
    """
    evaluator = Evaluator(headless=True, num_simulators=config.NUM_SIMULATORS)
    parent_selector = ParentSelector(offspring_size=config.OFFSPRING_SIZE, rng=rng)
    survivor_selector = SurvivorSelector(rng=rng)
    crossover_reproducer = CrossoverReproducer(
        rng=rng, innov_db_body=innov_db_body, innov_db_brain=innov_db_brain
    )

    modular_robot_evolution = ModularRobotEvolution(
        parent_selection=parent_selector,
        survivor_selection=survivor_selector,
        evaluator=evaluator,
        reproducer=crossover_reproducer,
    )

    # Create an initial population as we cant start from nothing.
    logging.info("Generating initial population.")
    initial_genotypes = [
        Genotype.random(
            innov_db_body=innov_db_body,
            innov_db_brain=innov_db_brain,
            rng=rng,
        )
        for _ in range(config.POPULATION_SIZE)
    ]

    # Evaluate the initial population.
    logging.info("Evaluating initial population.")
    initial_fitnesses = evaluator.evaluate(initial_genotypes)

    # Create a population of individuals, combining genotype with fitness.
    population = [
        Individual(genotype, fitness)
        for genotype, fitness in zip(initial_genotypes, initial_fitnesses, strict=True)
    ]

    # Save the best robot
    best_robot = find_best_robot(None, population)

    # Set the current generation to 0.
    generation_index = 0

    # Start the actual optimization process.
    logging.info("Start optimization process.")
    while generation_index < config.NUM_GENERATIONS:
        logging.info(f"Generation {generation_index + 1} / {config.NUM_GENERATIONS}.")

        """
        In contrast to the previous example we do not explicitly stat the order of operations here, but let the ModularRobotEvolution object do the scheduling.
        This does not give a performance boost, but is more readable and less prone to errors due to mixing up the order.
        
        Not that you are not restricted to the classical ModularRobotEvolution object, since you can adjust the step function as you want.
        """
        population = modular_robot_evolution.step(
            population
        )  # Step the evolution forward.

        # Find the new best robot
        best_robot = find_best_robot(best_robot, population)

        logging.info(f"Best robot until now: {best_robot.fitness}")
        logging.info(f"Genotype pickle: {pickle.dumps(best_robot)!r}")

        # Increase the generation index counter.
        generation_index += 1


if __name__ == "__main__":
    main()
