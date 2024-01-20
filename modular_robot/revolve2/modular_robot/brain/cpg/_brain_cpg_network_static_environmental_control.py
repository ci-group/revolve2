from .._brain_instance import BrainInstance
from ._brain_cpg_instance_environmental_control import (
    BrainCpgInstanceEnvironmentalControl,
)
from ._brain_cpg_network_static import BrainCpgNetworkStatic


class BrainCpgNSEnvironmentalControl(BrainCpgNetworkStatic):
    """A static cpg network with environment controlling."""

    def make_instance(self) -> BrainInstance:
        """
        Create an instance of this brain.

        :returns: The created instance.
        """
        return BrainCpgInstanceEnvironmentalControl(
            initial_state=self._initial_state,
            weight_matrix=self._weight_matrix,
            output_mapping=self._output_mapping,
        )
