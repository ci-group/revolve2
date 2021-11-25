from typing import List, Tuple


# TODO extend functionality and add error checking
class ActorControl:
    _dof_targets: List[Tuple[int, int, List[float]]]

    def __init__(self):
        self._dof_targets = []

    def set_dof_targets(
        self, environment: int, actor: int, targets: List[float]
    ) -> None:
        self._dof_targets.append((environment, actor, targets))
