from typing import List, Tuple, TypeVar, Union

from revolve2.core.database import Serializable
from typing_extensions import TypeGuard

from .._serializable_measures import SerializableMeasures
from ._pop_list import PopList

TIndividual = TypeVar("TIndividual", bound=Serializable)
TMeasures = TypeVar("TMeasures", bound=SerializableMeasures)


def _is_number_list(
    xs: List[Union[int, float, str, None]]
) -> TypeGuard[List[Tuple[int, float]]]:
    return all(isinstance(x, int) or isinstance(x, float) for x in xs)


def replace_if_better(
    original_population: PopList[TIndividual, TMeasures],
    offspring_population: PopList[TIndividual, TMeasures],
    measure: str,
) -> Tuple[List[int], List[int]]:
    """
    Compare each individual is offspring population with original population index-wise and replaces if better.

    Populations must be of the same size.

    :param original_population: The original population to replace individuals in. Will not be altered.
    :param offspring_population: The offspring population to take individuals from. Will not be unaltered. Individuals will be copied.
    :param measure: The measure to use for selection.
    :returns: Indices of the selected individuals in their respective populations. Original, offspring.
    """
    original_selection: List[int] = []
    offspring_selection: List[int] = []

    for index, (orig_ind, off_ind) in enumerate(
        zip(original_population, offspring_population)
    ):
        if off_ind.measures[measure] > orig_ind.measures[measure]:  # type: ignore # deal with this another time
            offspring_selection.append(index)
        else:
            original_selection.append(index)

    return original_selection, offspring_selection
