from abc import ABC, abstractmethod

from revolve2.simulation.scene import MultiBodySystem

from ._body_to_multi_body_system_mapping import BodyToMultiBodySystemMapping
from ._unbuilt_child import UnbuiltChild


class Builder(ABC):
    """An abstract builder class."""

    @abstractmethod
    def build(
        self,
        multi_body_system: MultiBodySystem,
        body_to_multi_body_system_mapping: BodyToMultiBodySystemMapping,
    ) -> list[UnbuiltChild]:
        """
        Build a module onto the Robot.

        :param multi_body_system: The multi body system of the robot.
        :param body_to_multi_body_system_mapping: A mapping from body to multi-body system
        :return: The next children to be built.
        """
