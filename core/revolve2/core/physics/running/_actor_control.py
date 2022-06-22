from typing import List, Tuple


# TODO extend functionality and add error checking
class ActorControl:
    _dof_targets: List[Tuple[int, List[float]]]  # actor, targets

    def __init__(self) -> None:
        self._dof_targets = []

    def set_dof_targets(self, actor: int, targets: List[float]) -> None:
        self._dof_targets.append((actor, targets))
