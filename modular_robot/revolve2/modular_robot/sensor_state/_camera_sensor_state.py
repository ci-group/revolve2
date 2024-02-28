from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray


class CameraSensorState(ABC):
    """The state of a camera sensor."""

    @property
    @abstractmethod
    def image(self) -> NDArray[np.float_]:
        """
        Get the current image.

        :returns: The image.
        """
        pass
