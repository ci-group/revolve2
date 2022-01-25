=============================================
A simple optimization process with a database
=============================================
The final results of this tutorial are available at ``<revolve2_source>/examples/simple_optimization>``.

-------------------
What you will learn
-------------------

* How to use the ``EvolutionaryOptimizer``
* How to use the ``Database``

-------------
Prerequisites
-------------

* Have Revolve2 :ref:`installed <installation/index:Installation>`.
* No supplementary libraries are required.
* Basic knowledge of evolutionary computing.

------------
Introduction
------------
You are going to create an evolutionary optimization process that can find a good solution to the `knapsack problem <https://en.wikipedia.org/wiki/Knapsack_problem>`_.
See also chapter 2.4.2 of `<https://www.cs.vu.nl/~gusz/ecbook/Eiben-Smith-Intro2EC-Ch2.pdf>`_.
The problem is as follows: given a set of items, each with a weight and a value, determine the items to include in a collection so that the total weight is less than or equal to a given limit and the total value is as large as possible.

The phenotype of a solution to the problem will be represented by a binary string.
Every bit corresponds to an item that can be included in the knapsack.
A one means the item is included, and a zero means it is not.
Naturally the sum of the weights of all items must be no larger than the maximum weight.
A genotype follows the same structure, but without the weight restriction.
The functions from genotype to phenotype simply adds items from left to right in the bit string, skipping any items that would bring the weight over the maximum.

The crossover operator will choose a random point along the bit string. All bits before that point will be taken taken from parent one and the rest from parent two.
The mutation operator will randomly flip each bit with 1/*n* chance, with *n* being the length of the bit strings.

--------------
Programming it
--------------
Start by creating a file called ``main.py``. For this tutorial, this is the only file you will need.

Begin by creating the basic objects for the optimization process.
To represent an item, create a class ``Item`` that contains a ``weight`` and a ``value``::

    from dataclasses import dataclass

    @dataclass
    class Item:
        weight: float
        value: float

A phenotype is simply a list of booleans::

    from typing import List

    # ...

    class Phenotype():
        items: List[bool]

        def __init__(self, items: List[bool]) -> None:
            self.items = items

The class for the Genotype is more complex.
It is also a list of booleans, but it should contain some methods.
Firstly, a ``random`` function that create a random genotype.
Secondly, a ``develop`` function that creates a phenotype from the genotype.
And thirdly, Revolve2 requires it to have ``serialize`` and ``deserialize`` functions that converts the object between ``StaticData``.
StaticData is Revolve2's type for JSON-like data containing only dictionaries, lists, bools, ints, floats, strings, bytes, and None::

    from __future__ import annotations
    from revolve2.core.database.serialization import Serializable
    from random import Random
    from revolve2.core.database import StaticData

    # ...

    class Genotype(Serializable):
        items: List[bool]

        def __init__(self, items: List[bool]) -> None:
            self.items = items

        @classmethod
        def random(cls, rng: Random, has_item_prob: float, num_items: int) -> Genotype:
            raise NotImplementedError()

        def develop(self, items: List[Item], maximum_weight: float) -> Phenotype:
            raise NotImplementedError()

        def serialize(self) -> StaticData:
            raise NotImplementedError()

        @classmethod
        def deserialize(cls, data: StaticData) -> Serializable:
            raise NotImplementedError()

A straightforward way to create a random bit string is to randomly assign each bit with a given probability::

    @classmethod
    def random(cls, rng: Random, has_item_prob: float, num_items: int) -> Genotype:
        return cls([rng.random() < has_item_prob for _ in range(num_items)])

``Serialize`` and ``Deserialize`` are very easy, because the contained ``items`` are already ``StaticData``::

    def serialize(self) -> StaticData:
        return self.items

    @classmethod
    def deserialize(cls, data: StaticData) -> Serializable:
        assert isinstance(data, list)
        assert all([isinstance(item, bool) for item in data])
        return cls(data)

Finally, the ``develop`` function adds items from the genotype's bit string from left to right, skipping any item that would make the knapsack too heavy::

    def develop(self, items: List[Item], maximum_weight: float) -> Phenotype:
        phenotype = []
        total_weight = 0
        for has_item, item in zip(self.items, items):
            if has_item and total_weight + item.weight < maximum_weight:
                phenotype.append(True)
            else:
                phenotype.append(False)

        return Phenotype(phenotype)

Now that the baseline object are there it is time to start with the actual optimization process.
Your program will use ``async`` Revolve2 functions, so the entry point for the program will be an async main::

    async def main() -> None:
        pass

    if __name__ == "__main__":
        import asyncio

        asyncio.run(main())

Define some parameters for the evolutionary algorithm::

    async def main() -> None:
        POPULATION_SIZE = 100
        OFFSPRING_SIZE = 100
        NUM_GENERATIONS = 25

        INITIAL_HAS_ITEM_PROB = 0.5

Revolve2 uses python's ``logging`` module. You can configure yourself what you want to do with the logs::

    import logging

    # ...

    async def main() -> None:
        # ...

        logging.basicConfig(
            level=logging.DEBUG,
            format="[%(asctime)s] [%(levelname)s] [%(module)s] %(message)s",
        )

        logging.info(f"Starting optimization")

It is important to have reproducible random number generation.
Revolve2 always uses a provided random object, never the global random object::

    async def main() -> None:
        # ...

        # random number generator
        rng = Random()
        rng.seed(100)

Create the problem itself. One hundred random items::

    async def main() -> None:
        # ...

        # create 100 random items
        items = [Item(rng.randrange(0, 100), rng.randrange(0, 100)) for _ in range(100)]

The first important thing is to create a database.
Revolve2 provides an interface for different database backends, but it is recommended to the the Sqlite backend::

    from revolve2.core.database.sqlite import Database as DbSqlite

    # ...

    async def main() -> None:
        # ...

        # database
        database = await DbSqlite.create(f"database")

Moving away from the ``main`` function, Revolve2 provides an ``EvolutionaryOptimizer`` class. To use it, inherit from it and fill in its abstract functions.
Additionally, it requires you to override ``init`` and call its ``asyncinit`` instead::

    from revolve2.core.optimization.ea import EvolutionaryOptimizer, Individual
    from revolve2.core.database import Database, Node

    # ...

    class Optimizer(EvolutionaryOptimizer[Genotype, float]):
        def __init__(self) -> None:
            pass

        async def create() -> Optimizer:
            self = Optimizer()
            # await super(Optimizer, self).asyncinit(...)
            # return self
            raise NotImplementedError()

        async def _evaluate_generation(
            self, individuals: List[Genotype], database: Database, dbview: Node
        ) -> List[float]:
            raise NotImplementedError()

        def _select_parents(
            self,
            generation: List[Individual[Genotype, float]],
            num_parents: int,
        ) -> List[List[Individual[Genotype, float]]]:
            raise NotImplementedError()

        def _select_survivors(
            self,
            old_individuals: List[Individual[Genotype, float]],
            new_individuals: List[Individual[Genotype, float]],
            num_survivors: int,
        ) -> List[Individual[Genotype, float]]:
            raise NotImplementedError()

        def _crossover(self, parents: List[Genotype]) -> Genotype:
            raise NotImplementedError()

        def _mutate(self, individual: Genotype) -> Genotype:
            raise NotImplementedError()

        def _must_do_next_gen(self) -> bool:
            raise NotImplementedError()

The ``EvolutionaryOptimizer`` base class has many arguments, but the names speak for themselves.
Importantly ``initial_fitness`` is an optional argument.
If set to None, the optimizer will automatically evaluate the initial population.
The second argument of ``EvolutionaryOptimizer``'s ``asyncinit`` is a database ``Node``.
Revolve's database structure is similar to JSON, and a ``Node`` points to an position in this structure.
This optimizer is the first and only part of the program touching the database, so it should write at the root node.
In addition to the required arguments, also add ``items``, representing all possible items, and ``num_generation``, the number of generations to perform the algorithm for.
These will be used later::

    from typing import Optional

    class Optimizer(EvolutionaryOptimizer[Genotype, float]):
        _items: List[Item]
        _num_generations: int

        # ...

        async def create(
            database: Database,
            rng: Random,
            population_size: int,
            offspring_size: int,
            initial_population: List[Genotype],
            initial_fitness: Optional[List[float]],
            items: List[Item],
            num_generations: int,
        ) -> Optimizer:
            self = Optimizer()

            await super(Optimizer, self).asyncinit(
                database,
                database.root,
                rng,
                population_size,
                offspring_size,
                initial_population,
                initial_fitness,
            )

            self._items = items
            self._num_generations = num_generations

            return self

The optimizer can now be added to the ``main`` function.
Create an initial population and start the optimizer.
Running this program should results in a raised ``NotImplementedError``::

    async def main() -> None:
        # ...

        initial_population = [
            Genotype.random(rng, INITIAL_HAS_ITEM_PROB, len(items))
            for _ in range(POPULATION_SIZE)
        ]

        ep = await Optimizer.create(
            database,
            rng=rng,
            population_size=POPULATION_SIZE,
            offspring_size=OFFSPRING_SIZE,
            initial_population=initial_population,
            initial_fitness=None,
            items=items,
            num_generations=NUM_GENERATIONS,
        )

        logging.info("Starting optimization process..")

        await ep.run()

        logging.info(f"Finished optimizing.")

Finally, you can now move on to the bread and butter of the program.
The ``EvolutionaryOptimizer`` asks you to evaluate a complete generation at once.
For some problems this can be beneficial in terms of concurrency, but for this example it does not matter.
The evaluation of an individual is defined to be the total value of its contained items::

    async def _evaluate_generation(
        self, individuals: List[Genotype], database: Database, dbview: Node
    ) -> List[float]:
        return [
            float(
                sum(
                    [
                        has_items * item.value
                        for has_items, item in zip(individual.items, self._items)
                    ]
                )
            )
            for individual in individuals
        ]

After each generation, and, in the current state of the program, after evaluating the initial population, the ``EvolutionaryOptimizer`` ask you if it should continue with another generation.
You already have the number of generations to evaluate, so this is straightforward::

    def _must_do_next_gen(self) -> bool:
        return self.generation_index != self._num_generations

The optimizer is now stuck at selecting parent for the new generation.
It expects you to return a list of groups of parent, each of which will make children.
You can use any selection function that you want, but this tutorial selects pairs of parents using a tournament with two participants::

    import revolve2.core.optimization.ea.selection as selection

    # ...

    def _select_parents(
        self,
        generation: List[Individual[Genotype, float]],
        num_parents: int,
    ) -> List[List[Individual[Genotype, float]]]:
        return [
            [
                i[0]
                for i in selection.multiple_unique(
                    [(i, i.fitness) for i in generation],
                    2,
                    lambda gen: selection.tournament(self._rng, gen, k=2),
                )
            ]
            for _ in range(num_parents)
        ]

After parent selection comes crossover and mutation.
Fill in these functions using the definitions from the introduction of this tutorial::

    def _crossover(self, parents: List[Genotype]) -> Genotype:
        assert len(parents) == 2
        point = self._rng.randrange(0, len(parents[0].items))
        return Genotype(parents[0].items[0:point] + parents[1].items[point:])

    def _mutate(self, individual: Genotype) -> Genotype:
        return Genotype(
            [
                has_item ^ (self._rng.random() < 1 / len(individual.items))
                for has_item in individual.items
            ]
        )

Then the last thing is survivor selection.
This tutorial uses a two participant tournament with a steady state population, meaning the previous generation is also allowed to participate in the tournament::

    import revolve2.core.optimization.ea.population_management as population_management

    # ...

    def _select_survivors(
        self,
        old_individuals: List[Individual[Genotype, float]],
        new_individuals: List[Individual[Genotype, float]],
        num_survivors: int,
    ) -> List[Individual[Genotype, float]]:
        assert len(old_individuals) == num_survivors

        return [
            i[0]
            for i in population_management.steady_state(
                [(i, i.fitness) for i in old_individuals],
                [(i, i.fitness) for i in new_individuals],
                lambda pop: selection.tournament(self._rng, pop, k=2),
            )
        ]

After all this, your optimization program is finally ready.

---------------------
Running and analyzing
---------------------
Looking at the log of this program, you will see that it creates 25 generations.
It is not very fast, because the ``EvolutionaryOptimizer`` assumes your evaluation is an expensive function, and writes to the databases every generation to allow for recovery.
You will see that if you manually restart the program in the middle of the optimization process it will restart at the last successfully evaluated generation.

Of course you want to see the results of the program.
The ``EvolutionaryOptimizer`` comes with an ``Analyzer``.
This class knows exactly what the database structure used by the optimizer is.
However, for simplicity this tutorial is going to use the analysis program ``plot_ea_fitness.py``, which can plot fitness over all generations::

    python core/revolve2/analysis/core/plot_ea_fitness.py database

If this tool is slow, this is something that is actively being worked on. Your output should look similar to the image below.

.. image:: simple_optimization_fitness_plot.png
    :width: 100%

------------
Contributing
------------
This marks the end of this tutorial. Feedback and contributions are welcome at Revolve's code repository.