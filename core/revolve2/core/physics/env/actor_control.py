from typing import Mapping


# TODO extend functionality and add error checking
class ActorControl:
    _position_targets: Mapping[str, Mapping[str, Mapping[str, float]]]

    def __init__(self):
        self._position_targets = {}

    def set_position_targets(
        self, environment: str, actor: str, targets: Mapping[str, float]
    ) -> None:
        if environment not in self._position_targets:
            self._position_targets[environment] = {}
        self._position_targets[environment][actor] = targets
