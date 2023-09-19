"""Standard modular robots."""
import numpy as np
from revolve2.modular_robot import ActiveHinge, Body, Brick
from revolve2.modular_robot.v2._property_set import V2PropertySet


def all() -> list[Body]:
    """
    Get a list of all standard module robots.

    :returns: The list of robots.
    """
    return [
        gecko_v2(),
    ]


def get(name: str) -> Body:
    """
    Get a robot by name.

    :param name: The name of the robot to get.
    :returns: The robot with that name.
    :raises ValueError: When a robot with that name does not exist.
    """
    case = {
        "gecko_v2": gecko_v2(),
    }

    try:
        return case[name]

    except:
        raise ValueError(f"Robot does not exist: {name}")


def gecko_v2() -> Body:
    """
    Sample robot with new HW config.

    all previously designed robots can be translated to the new hardware using Body(new_hardware=True)
    the position for core-connected modules is optional, by default pos 5 (middle of core)

    :returns: the robot
    """
    body = Body(V2PropertySet())
    body.core.right = ActiveHinge(0.0, attachment_position=8)
    body.core.right.attachment = Brick(0.0)

    body.core.left = ActiveHinge(0.0, attachment_position=8)
    body.core.left.attachment = Brick(0.0)

    body.core.back = ActiveHinge(np.pi / 2.0, attachment_position=8)
    body.core.back.attachment = Brick(-np.pi / 2.0)
    body.core.back.attachment.front = ActiveHinge(np.pi / 2.0)
    body.core.back.attachment.front.attachment = Brick(-np.pi / 2.0)
    body.core.back.attachment.front.attachment.left = ActiveHinge(0.0)
    body.core.back.attachment.front.attachment.right = ActiveHinge(0.0)
    body.core.back.attachment.front.attachment.left.attachment = Brick(0.0)
    body.core.back.attachment.front.attachment.right.attachment = Brick(0.0)

    body.finalize()

    return body
