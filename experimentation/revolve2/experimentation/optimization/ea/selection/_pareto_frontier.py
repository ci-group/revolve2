from itertools import combinations
from typing import TypeVar

import numpy as np
from numpy.typing import NDArray

TValues = TypeVar("TValues")
TOther = TypeVar("TOther")


def pareto_frontier(
    frontier_values: list[list[TValues]], frontier_order: list[bool], to_take: int
) -> list[int]:
    """
    Return individuals based on their respective frontier values and their domination order.

    For mor information on the pareto frontier check: https://en.wikipedia.org/wiki/Pareto_front.

    :param frontier_values: Lists of values that are used for the frontier. The order of the list represents the importance of a value in descending order. These values need to be numeric.
    :param frontier_order: List of orders for the values used in frontier selection. True = ascending, False = descending.
    :param to_take: The amount of individuals to return from the frontier.
    :returns: The index of the individuals that were selected. Returned in descending order, wrt. frontier values and domination order (Best is first).
    """
    assert all(len(x) == len(y) for x, y in combinations(frontier_values, 2))
    assert len(frontier_values[0]) >= to_take

    domination_orders = _get_domination_orders(
        np.array(frontier_values, dtype=np.float64).T, frontier_order
    )

    all_values = np.array([domination_orders] + frontier_values, dtype=np.float64)
    frontier_order = [False] + frontier_order
    """Domination order is descending by default. The more individuals are dominated by a target, the better."""

    srt_array = [
        all_values[i] if frontier_order[i] else -all_values[i]
        for i in range(len(frontier_values) + 1)
    ]
    srt = np.lexsort(srt_array[:-1])
    """Lexical sort needs reversed order, where the most important feature is sorted last."""
    return list(srt[:to_take])


def _get_domination_orders(
    value_array: NDArray[np.float_], frontier_order: list[bool]
) -> list[int]:
    """
    Find the pareto domination order for each point.

    :param value_array: A (*value) array.
    :param frontier_order: The order of a specific value.
    :return: A mask for the pareto front.
    """
    domination_orders = [0] * value_array.shape[0]

    i = 0
    for pair in value_array:
        dom = sum(
            [
                any(
                    [
                        p[j] > pair[j] if frontier_order[j] else p[j] < pair[j]
                        for j in range(len(frontier_order))
                    ]
                )
                for p in value_array
            ]
        )
        domination_orders[i] = dom
        i += 1
    return domination_orders
