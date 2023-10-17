from ._render import Render
from revolve2.modular_robot import ModularRobot, MorphologicalMeasures

from pathlib import Path


def render_robot(robot: ModularRobot, path: Path) -> None:
    body = robot.body
    if not MorphologicalMeasures(body).is_2d:
        raise ValueError("robot is not 2d")

    Render().render_robot(body.core, path)
