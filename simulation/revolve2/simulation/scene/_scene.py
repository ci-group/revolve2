from dataclasses import dataclass, field

from ._multi_body_system import MultiBodySystem
from ._simulation_handler import SimulationHandler


@dataclass(kw_only=True)
class Scene:
    """Description of a scene that can be simulated."""

    handler: SimulationHandler
    _multi_body_systems: list[MultiBodySystem] = field(default_factory=list, init=False)
    """
    Multi-body system in this scene.

    Don't add to this directly, but use `add_multi_body_system` instead.
    """

    def add_multi_body_system(self, multi_body_system: MultiBodySystem) -> None:
        """
        Add a multi-body system to the scene.

        :param multi_body_system: The multi-body system to add.
        """
        self._multi_body_systems.append(multi_body_system)

    @property
    def multi_body_systems(self) -> list[MultiBodySystem]:
        """
        Get the multi-body systems in scene.

        Do not make changes to this list.

        :returns: The multi-body systems in the scene.
        """
        return self._multi_body_systems[:]
