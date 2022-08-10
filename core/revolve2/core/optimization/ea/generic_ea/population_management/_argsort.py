from typing import List, Sequence, TypeVar

Item = TypeVar("Item")  # TODO bound too less than and/or greater than


def argsort(seq: Sequence[Item]) -> List[int]:
    """
    Get the indices of the sequence sorted by value.

    :param seq: The sequence.
    :returns: The indices.
    """
    # http://stackoverflow.com/questions/3071415/efficient-method-to-calculate-the-rank-vector-of-a-list-in-python
    return sorted(range(len(seq)), key=seq.__getitem__)  # type: ignore #TODO
