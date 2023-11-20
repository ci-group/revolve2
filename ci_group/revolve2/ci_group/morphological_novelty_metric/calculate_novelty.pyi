import numpy as np
from numpy.typing import NDArray

"""Allow mypy and sphinx to resolve the compiled cython module."""

def calculate_novelty(
    histograms: NDArray[np.int64], amount_instances: int, histogram_size: int
) -> NDArray[np.float64]: ...
