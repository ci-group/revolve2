"""Functions for standardized number generation."""

import hashlib

import numpy as np


def seed_from_string(text: str) -> int:
    """
    Convert a string seed to an integer seed.

    :param text: The seed as string.
    :returns: The seed as integer.
    """
    return int(
        hashlib.sha256(text.encode()).hexdigest(),
        16,
    )


def make_rng(seed: int) -> np.random.Generator:
    """
    Create a numpy random number generator from a seed.

    :param seed: The seed to use.
    :returns: The random number generator.
    """
    return np.random.Generator(np.random.PCG64(seed))
