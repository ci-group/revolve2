from ._render import Render
from revolve2.modular_robot import ModularRobot

from pathlib import Path


def render_robot(robot: ModularRobot, path: Path) -> None:
    Render().render_robot(robot.body.core, path)
