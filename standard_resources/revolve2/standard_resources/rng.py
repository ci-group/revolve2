"""Functions for standardized number generation."""

import hashlib
import logging
from datetime import datetime

import numpy as np


def seed_from_time() -> int:
    """
    Create a seed from the current time in microseconds.

    :returns: The created seed.
    """
    return int(datetime.now().timestamp() * 1e6)


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


def make_rng_time_seed(log_seed: bool = True) -> np.random.Generator:
    """
    Create a numpy random number generator from a seed.

    :param log_seed: If automatically created seed should be logged. It probably should.
    :returns: The random number generator.
    """
    seed = seed_from_time()
    if log_seed:
        logging.info(f"Rng seed: {seed}")
    return np.random.Generator(np.random.PCG64(seed))
