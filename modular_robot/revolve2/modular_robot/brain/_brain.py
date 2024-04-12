from abc import ABC, abstractmethod

from ._brain_instance import BrainInstance


class Brain(ABC):
    """
    The brain of a modular robot.

    Inherit from this to implement your own brain.
    Each brain implements the `make_instance` function,
    which create the actual brain instance that control the robot.
    The instance contains all the state associated with the control strategy;
    this class must be stateless.
    """

    @abstractmethod
    def make_instance(self) -> BrainInstance:
        """
        Create an instance of this brain.

        :returns: The created instance.
        """
