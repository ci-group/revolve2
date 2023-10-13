from dataclasses import dataclass


@dataclass
class BodyId:
    """Id of a MuJoCo body."""

    id: int
