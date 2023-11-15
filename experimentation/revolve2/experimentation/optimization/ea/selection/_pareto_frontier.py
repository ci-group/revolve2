from typing import TypeVar
import numpy as np
from numpy.typing import NDArray
from itertools import product, combinations

TIndividual = TypeVar('TIndividual')
TValues = TypeVar("TValues")
TOther = TypeVar("TOther")


def pareto_frontier(frontier_values: list[list[TValues]], frontier_order: list[bool], to_take: int) -> list[int]:
    """
    Perform tournament selection and return the index of the best individual.

    :param frontier_values: List of lists that are used for the frontier. The order of the lists represents the importance of a value -> first most important, last -> least important.
    :param frontier_order: List of orders for the frontier selection (descending/ ascending). True if descending.
    :param to_take: The amount of individuals to return from the frontier.
    :returns: The index of te individual that won the tournament.
    """

    assert all(len(x) == len(y) for x, y in combinations(frontier_values, 2))
    assert len(frontier_values[0]) >= to_take

    domination_orders = _get_domination_order(np.array(frontier_values, dtype=np.float64).T, frontier_order)
    all_values = np.array([domination_orders] + frontier_values, dtype=np.float64)

    frontier_order = [False] + frontier_order  # adding order for domination order -> ascending.
    srt_array = [all_values[i] if frontier_order[i] else -all_values[i] for i in range(len(frontier_values)+1)]
    srt = np.lexsort(srt_array[:-1])  # lexical sort needs reversed order -> most important last.

    indices = np.arange(len(frontier_values[0]))[srt]
    return indices[:to_take].tolist()


def _get_domination_order(pairs: NDArray[np.float_], frontier_order: list[bool]) -> list[int]:
    """
    Find the pareto domination order for each point.

    :param pairs: A (*value) array.
    :return: A mask for the pareto front.
    """
    dom_orders = [0] * pairs.shape[0]

    i = 0
    for pair in pairs:
        dom = sum(
            [
                any(
                    [
                        p[j] > pair[j] if frontier_order[j] else p[j] < pair[j]
                        for j in range(len(frontier_order))
                    ]
                )
                for p in pairs
            ]
        )
        dom_orders[i] = dom
        i += 1
    return dom_orders
