"""Functions for standardized number generation."""

import numpy as np
import hashlib


def seed_from_string(text: str) -> int:
    int(
        hashlib.sha256(text.encode()).hexdigest(),
        16,
    )


def make_rng(seed: int) -> np.random.Generator:
    return np.random.Generator(np.random.PCG64(seed))
