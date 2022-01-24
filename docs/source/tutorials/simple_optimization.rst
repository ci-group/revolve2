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
