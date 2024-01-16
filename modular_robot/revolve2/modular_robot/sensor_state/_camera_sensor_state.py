from abc import ABC, abstractmethod

import numpy as np
from numpy.typing import NDArray


class CameraSensorState(ABC):
    """The state of an active hinge sensor."""

    @property
    @abstractmethod
    def image(self) -> NDArray[np.int_]:
        """
        Get the current image.

        :returns: The image.
        """
        pass
