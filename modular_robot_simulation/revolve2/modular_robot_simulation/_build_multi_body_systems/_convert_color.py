from revolve2.modular_robot.body import Color
from revolve2.simulation.scene import Color as SimulationColor


def convert_color(color: Color) -> SimulationColor:
    """
    Convert ModularRobot Color to Simulator Color.

    :param color: The ModularRobot color.
    :return: The Simulator color.
    """
    return SimulationColor(color.red, color.green, color.blue, color.alpha)
