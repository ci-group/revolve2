from typing import TypeVar
import numpy as np
from numpy.typing import NDArray

TIndividual = TypeVar('TIndividual')
TFitness = TypeVar("TFitness")
TOther = TypeVar("TOther")


def pareto_frontier(individuals: list[TIndividual], fitnesses: list[TFitness], others: list[TOther], to_take: int) -> list[int]:
    """
    Perform tournament selection and return the index of the best individual.

    :param individuals: List of individuals that can form the frontier.
    :param fitnesses: List of finesses of individuals that are used for the frontier.
    :param others: List of other values of individuals for the frontier selection.
    :param to_take: The amount of individuals to return from the frontier.
    :returns: The index of te individual that won the tournament.
    """

    assert len(individuals) == len(fitnesses) == len(others)
    assert len(fitnesses) >= to_take

    domination_orders = _get_domination_order(np.array([fitnesses, others], dtype=np.float64).T)
    all_values = np.array([domination_orders, fitnesses, others], dtype=np.float64)
    srt = np.lexsort([all_values[2], -all_values[1], -all_values[0]])

    indices = np.arange(len(fitnesses))[srt]
    return indices[:to_take].tolist()


def _get_domination_order(pairs: NDArray(np.float_)) -> NDArray(np.int_):
    """
    Find the pareto domination order for each point.

    :param pairs: A (fitness, age) array.
    :return: A mask for the pareto front.
    """
    dom_orders = [0] * pairs.shape[0]

    i = 0
    for pair in pairs:
        dom = sum([p[0] < pair[0] or p[1] > pair[1] for p in pairs])
        """We dominate another point p if our fitness is higher or age is lower."""
        dom_orders[i] = dom
        i += 1
    return dom_orders
