from .._brain import Brain
from .._brain_instance import BrainInstance
from ._brain_dummy_instance import BrainDummyInstance


class BrainDummy(Brain):
    """A brain that does nothing."""

    def make_instance(self) -> BrainInstance:
        """
        Create an instance of this brain.

        :returns: The created instance.
        """
        return BrainDummyInstance()
