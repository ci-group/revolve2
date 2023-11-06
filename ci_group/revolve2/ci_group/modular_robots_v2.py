"""Standard modular robots."""
import numpy as np

from revolve2.modular_robot.body.v2 import ActiveHingeV2, BodyV2, BrickV2


def all() -> list[BodyV2]:
    """
    Get a list of all standard module robots.

    :returns: The list of robots.
    """
    return [
        gecko_v2(),
    ]


def get(name: str) -> BodyV2:
    """
    Get a robot by name.

    :param name: The name of the robot to get.
    :returns: The robot with that name.
    :raises ValueError: When a robot with that name does not exist.
    """
    match name:
        case "gecko":
            return gecko_v2()
        case _:
            raise ValueError(f"Robot does not exist: {name}")


def gecko_v2() -> BodyV2:
    """
    Sample robot with new HW config.

    all previously designed robots can be translated to the new hardware using Body(new_hardware=True)
    the position for core-connected modules is optional, by default pos 5 (middle of core)

    :returns: the robot
    """
    body = BodyV2()

    body.core.right.bottom = ActiveHingeV2(0.0)
    body.core.right.bottom.attachment = BrickV2(0.0)

    body.core.left.bottom = ActiveHingeV2(0.0)
    body.core.left.bottom.attachment = BrickV2(0.0)

    body.core.back.bottom = ActiveHingeV2(np.pi / 2.0)
    body.core.back.bottom.attachment = BrickV2(-np.pi / 2.0)
    body.core.back.bottom.attachment.front = ActiveHingeV2(np.pi / 2.0)
    body.core.back.bottom.attachment.front.attachment = BrickV2(-np.pi / 2.0)
    body.core.back.bottom.attachment.front.attachment.left = ActiveHingeV2(0.0)
    body.core.back.bottom.attachment.front.attachment.right = ActiveHingeV2(0.0)
    body.core.back.bottom.attachment.front.attachment.left.attachment = BrickV2(0.0)
    body.core.back.bottom.attachment.front.attachment.right.attachment = BrickV2(0.0)

    return body
