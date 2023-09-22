"""Standard modular robots."""
import numpy as np
from revolve2.modular_robot import ActiveHinge, Body, Brick
from revolve2.modular_robot.v1._property_set import V1PropertySet

_PROPERTIES = V1PropertySet()


def all() -> list[Body]:
    """
    Get a list of all standard module robots.

    :returns: The list of robots.
    """
    return [
        babya_v1(),
        babyb_v1(),
        blokky_v1(),
        garrix_v1(),
        gecko_v1(),
        insect_v1(),
        linkin_v1(),
        longleg_v1(),
        penguin_v1(),
        pentapod_v1(),
        queen_v1(),
        salamander_v1(),
        squarish_v1(),
        snake_v1(),
        spider_v1(),
        stingray_v1(),
        tinlicker_v1(),
        turtle_v1(),
        ww_v1(),
        zappa_v1(),
        ant_v1(),
        park_v1(),
    ]


def get(name: str) -> Body:
    """
    Get a robot by name.

    :param name: The name of the robot to get.
    :returns: The robot with that name.
    :raises ValueError: When a robot with that name does not exist.
    """
    case = {
        "gecko": gecko_v1,
        "spider": spider_v1,
        "babya": babya_v1,
        "ant": ant_v1,
        "salamander": salamander_v1,
        "blokky": blokky_v1,
        "park": park_v1,
        "babyb": babyb_v1,
        "garrix": garrix_v1,
        "insect": insect_v1,
        "linkin": linkin_v1,
        "longleg": longleg_v1,
        "penguin": penguin_v1,
        "pentapod": pentapod_v1,
        "queen": queen_v1,
        "squarish": squarish_v1,
        "snake": snake_v1,
        "stingray": stingray_v1,
        "tinlicker": tinlicker_v1,
        "turtle": turtle_v1,
        "ww": ww_v1,
        "zappa": zappa_v1,
    }

    try:
        return case[name]()
    except:
        raise ValueError(f"Robot does not exist: {name}")


def spider_v1() -> Body:
    """
    Get the spider modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.left = ActiveHinge(np.pi / 2.0)
    body.core.left.attachment = Brick(-np.pi / 2.0)
    body.core.left.attachment.front = ActiveHinge(0.0)
    body.core.left.attachment.front.attachment = Brick(0.0)

    body.core.right = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment = Brick(-np.pi / 2.0)
    body.core.right.attachment.front = ActiveHinge(0.0)
    body.core.right.attachment.front.attachment = Brick(0.0)

    body.core.front = ActiveHinge(np.pi / 2.0)
    body.core.front.attachment = Brick(-np.pi / 2.0)
    body.core.front.attachment.front = ActiveHinge(0.0)
    body.core.front.attachment.front.attachment = Brick(0.0)

    body.core.back = ActiveHinge(np.pi / 2.0)
    body.core.back.attachment = Brick(-np.pi / 2.0)
    body.core.back.attachment.front = ActiveHinge(0.0)
    body.core.back.attachment.front.attachment = Brick(0.0)

    body.finalize()
    return body


def gecko_v1() -> Body:
    """
    Get the gecko modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.left = ActiveHinge(0.0)
    body.core.left.attachment = Brick(0.0)

    body.core.right = ActiveHinge(0.0)
    body.core.right.attachment = Brick(0.0)

    body.core.back = ActiveHinge(np.pi / 2.0)
    body.core.back.attachment = Brick(-np.pi / 2.0)
    body.core.back.attachment.front = ActiveHinge(np.pi / 2.0)
    body.core.back.attachment.front.attachment = Brick(-np.pi / 2.0)
    body.core.back.attachment.front.attachment.left = ActiveHinge(0.0)
    body.core.back.attachment.front.attachment.left.attachment = Brick(0.0)
    body.core.back.attachment.front.attachment.right = ActiveHinge(0.0)
    body.core.back.attachment.front.attachment.right.attachment = Brick(0.0)

    body.finalize()
    return body


def babya_v1() -> Body:
    """
    Get the babya modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.left = ActiveHinge(0.0)
    body.core.left.attachment = Brick(0.0)

    body.core.right = ActiveHinge(0.0)
    body.core.right.attachment = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment.attachment = Brick(-np.pi / 2.0)
    body.core.right.attachment.attachment.front = ActiveHinge(0.0)
    body.core.right.attachment.attachment.front.attachment = Brick(0.0)

    body.core.back = ActiveHinge(np.pi / 2.0)
    body.core.back.attachment = Brick(-np.pi / 2.0)
    body.core.back.attachment.front = ActiveHinge(np.pi / 2.0)
    body.core.back.attachment.front.attachment = Brick(-np.pi / 2.0)
    body.core.back.attachment.front.attachment.left = ActiveHinge(0.0)
    body.core.back.attachment.front.attachment.left.attachment = Brick(0.0)
    body.core.back.attachment.front.attachment.right = ActiveHinge(0.0)
    body.core.back.attachment.front.attachment.right.attachment = Brick(0.0)

    body.finalize()
    return body


def ant_v1() -> Body:
    """
    Get the ant modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.left = ActiveHinge(0.0)
    body.core.left.attachment = Brick(0.0)

    body.core.right = ActiveHinge(0.0)
    body.core.right.attachment = Brick(0.0)

    body.core.back = ActiveHinge(np.pi / 2.0)
    body.core.back.attachment = Brick(-np.pi / 2.0)
    body.core.back.attachment.left = ActiveHinge(0.0)
    body.core.back.attachment.left.attachment = Brick(0.0)
    body.core.back.attachment.right = ActiveHinge(0.0)
    body.core.back.attachment.right.attachment = Brick(0.0)

    body.core.back.attachment.front = ActiveHinge(np.pi / 2.0)
    body.core.back.attachment.front.attachment = Brick(-np.pi / 2.0)
    body.core.back.attachment.front.attachment.left = ActiveHinge(0.0)
    body.core.back.attachment.front.attachment.left.attachment = Brick(0.0)
    body.core.back.attachment.front.attachment.right = ActiveHinge(0.0)
    body.core.back.attachment.front.attachment.right.attachment = Brick(0.0)

    body.finalize()
    return body


def salamander_v1() -> Body:
    """
    Get the salamander modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.left = ActiveHinge(np.pi / 2.0)
    body.core.left.attachment = ActiveHinge(-np.pi / 2.0)

    body.core.right = ActiveHinge(0.0)

    body.core.back = ActiveHinge(np.pi / 2.0)
    body.core.back.attachment = Brick(-np.pi / 2.0)
    body.core.back.attachment.left = ActiveHinge(0.0)
    body.core.back.attachment.front = Brick(0.0)
    body.core.back.attachment.front.left = ActiveHinge(0.0)
    body.core.back.attachment.front.front = ActiveHinge(np.pi / 2.0)
    body.core.back.attachment.front.front.attachment = Brick(-np.pi / 2.0)

    body.core.back.attachment.front.front.attachment.left = ActiveHinge(0.0)
    body.core.back.attachment.front.front.attachment.left.attachment = Brick(0.0)
    body.core.back.attachment.front.front.attachment.left.attachment.left = Brick(0.0)
    body.core.back.attachment.front.front.attachment.left.attachment.front = (
        ActiveHinge(np.pi / 2.0)
    )
    body.core.back.attachment.front.front.attachment.left.attachment.front.attachment = ActiveHinge(
        -np.pi / 2.0
    )

    body.core.back.attachment.front.front.attachment.front = Brick(0.0)
    body.core.back.attachment.front.front.attachment.front.left = ActiveHinge(0.0)
    body.core.back.attachment.front.front.attachment.front.front = Brick(0.0)
    body.core.back.attachment.front.front.attachment.front.front.left = ActiveHinge(0.0)
    body.core.back.attachment.front.front.attachment.front.front.front = Brick(0.0)
    body.core.back.attachment.front.front.attachment.front.front.front.front = (
        ActiveHinge(np.pi / 2.0)
    )
    body.core.back.attachment.front.front.attachment.front.front.front.front.attachment = Brick(
        -np.pi / 2.0
    )
    body.core.back.attachment.front.front.attachment.front.front.front.front.attachment.left = Brick(
        0.0
    )
    body.core.back.attachment.front.front.attachment.front.front.front.front.attachment.front = ActiveHinge(
        np.pi / 2.0
    )

    body.finalize()
    return body


def blokky_v1() -> Body:
    """
    Get the blokky modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.left = ActiveHinge(np.pi / 2.0)
    body.core.back = Brick(0.0)
    body.core.back.right = ActiveHinge(np.pi / 2.0)
    body.core.back.front = ActiveHinge(np.pi / 2.0)
    body.core.back.front.attachment = ActiveHinge(-np.pi / 2.0)
    body.core.back.front.attachment.attachment = Brick(0.0)
    body.core.back.front.attachment.attachment.front = Brick(0.0)
    body.core.back.front.attachment.attachment.front.right = Brick(0.0)
    body.core.back.front.attachment.attachment.front.right.left = Brick(0.0)
    body.core.back.front.attachment.attachment.front.right.front = Brick(0.0)
    body.core.back.front.attachment.attachment.right = Brick(0.0)
    body.core.back.front.attachment.attachment.right.front = Brick(0.0)
    body.core.back.front.attachment.attachment.right.front.right = Brick(0.0)
    body.core.back.front.attachment.attachment.right.front.front = ActiveHinge(0.0)

    body.finalize()
    return body


def park_v1() -> Body:
    """
    Get the park modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.back = ActiveHinge(np.pi / 2.0)
    body.core.back.attachment = ActiveHinge(-np.pi / 2.0)
    body.core.back.attachment.attachment = Brick(0.0)
    body.core.back.attachment.attachment.right = Brick(0.0)
    body.core.back.attachment.attachment.left = ActiveHinge(0.0)
    body.core.back.attachment.attachment.front = Brick(0.0)
    body.core.back.attachment.attachment.front.right = ActiveHinge(-np.pi / 2.0)
    body.core.back.attachment.attachment.front.front = ActiveHinge(-np.pi / 2.0)
    body.core.back.attachment.attachment.front.left = ActiveHinge(0.0)
    body.core.back.attachment.attachment.front.left.attachment = Brick(0.0)
    body.core.back.attachment.attachment.front.left.attachment.right = ActiveHinge(
        -np.pi / 2.0
    )
    body.core.back.attachment.attachment.front.left.attachment.front = Brick(0.0)
    body.core.back.attachment.attachment.front.left.attachment.front = ActiveHinge(0.0)
    body.core.back.attachment.attachment.front.left.attachment.front.attachment = Brick(
        0.0
    )

    body.finalize()
    return body


def babyb_v1() -> Body:
    """
    Get the babyb modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.left = ActiveHinge(np.pi / 2.0)
    body.core.left.attachment = Brick(-np.pi / 2.0)
    body.core.left.attachment.front = ActiveHinge(0.0)
    body.core.left.attachment.front.attachment = Brick(0.0)
    body.core.left.attachment.front.attachment.front = ActiveHinge(np.pi / 2.0)
    body.core.left.attachment.front.attachment.front.attachment = Brick(0.0)

    body.core.right = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment = Brick(-np.pi / 2.0)
    body.core.right.attachment.front = ActiveHinge(0.0)
    body.core.right.attachment.front.attachment = Brick(0.0)
    body.core.right.attachment.front.attachment.front = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment.front.attachment.front.attachment = Brick(0.0)

    body.core.front = ActiveHinge(np.pi / 2.0)
    body.core.front.attachment = Brick(-np.pi / 2.0)
    body.core.front.attachment.front = ActiveHinge(0.0)
    body.core.front.attachment.front.attachment = Brick(0.0)
    body.core.front.attachment.front.attachment.front = ActiveHinge(np.pi / 2.0)
    body.core.front.attachment.front.attachment.front.attachment = Brick(0.0)

    body.core.back = ActiveHinge(np.pi / 2.0)
    body.core.back.attachment = Brick(-np.pi / 2.0)

    body.finalize()
    return body


def garrix_v1() -> Body:
    """
    Get the garrix modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.front = ActiveHinge(np.pi / 2.0)

    body.core.left = ActiveHinge(np.pi / 2.0)
    body.core.left.attachment = ActiveHinge(0.0)
    body.core.left.attachment.attachment = ActiveHinge(-np.pi / 2.0)
    body.core.left.attachment.attachment.attachment = Brick(0.0)
    body.core.left.attachment.attachment.attachment.front = Brick(0.0)
    body.core.left.attachment.attachment.attachment.left = ActiveHinge(0.0)

    part2 = Brick(0.0)
    part2.right = ActiveHinge(np.pi / 2.0)
    part2.front = ActiveHinge(np.pi / 2.0)
    part2.left = ActiveHinge(0.0)
    part2.left.attachment = ActiveHinge(np.pi / 2.0)
    part2.left.attachment.attachment = ActiveHinge(-np.pi / 2.0)
    part2.left.attachment.attachment.attachment = Brick(0.0)

    body.core.left.attachment.attachment.attachment.left.attachment = part2

    body.finalize()
    return body


def insect_v1() -> Body:
    """
    Get the insect modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.right = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment = ActiveHinge(-np.pi / 2.0)
    body.core.right.attachment.attachment = Brick(0.0)
    body.core.right.attachment.attachment.right = ActiveHinge(0.0)
    body.core.right.attachment.attachment.front = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment.attachment.left = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment.attachment.left.attachment = Brick(-np.pi / 2.0)
    body.core.right.attachment.attachment.left.attachment.front = ActiveHinge(
        np.pi / 2.0
    )
    body.core.right.attachment.attachment.left.attachment.right = ActiveHinge(0.0)
    body.core.right.attachment.attachment.left.attachment.right.attachment = (
        ActiveHinge(0.0)
    )
    body.core.right.attachment.attachment.left.attachment.right.attachment.attachment = ActiveHinge(
        np.pi / 2.0
    )

    body.finalize()
    return body


def linkin_v1() -> Body:
    """
    Get the linkin modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.back = ActiveHinge(0.0)

    body.core.right = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment = ActiveHinge(0.0)
    body.core.right.attachment.attachment = ActiveHinge(0.0)
    body.core.right.attachment.attachment.attachment = ActiveHinge(-np.pi / 2.0)
    body.core.right.attachment.attachment.attachment.attachment = Brick(0.0)

    part2 = body.core.right.attachment.attachment.attachment.attachment
    part2.front = Brick(0.0)

    part2.left = ActiveHinge(0.0)
    part2.left.attachment = ActiveHinge(0.0)

    part2.right = ActiveHinge(np.pi / 2.0)
    part2.right.attachment = ActiveHinge(-np.pi / 2.0)
    part2.right.attachment.attachment = ActiveHinge(0.0)
    part2.right.attachment.attachment.attachment = ActiveHinge(np.pi / 2.0)
    part2.right.attachment.attachment.attachment.attachment = ActiveHinge(0.0)

    body.finalize()
    return body


def longleg_v1() -> Body:
    """
    Get the longleg modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.left = ActiveHinge(np.pi / 2.0)
    body.core.left.attachment = ActiveHinge(0.0)
    body.core.left.attachment.attachment = ActiveHinge(0.0)
    body.core.left.attachment.attachment.attachment = ActiveHinge(-np.pi / 2.0)
    body.core.left.attachment.attachment.attachment.attachment = ActiveHinge(0.0)
    body.core.left.attachment.attachment.attachment.attachment.attachment = Brick(0.0)

    part2 = body.core.left.attachment.attachment.attachment.attachment.attachment
    part2.right = ActiveHinge(0.0)
    part2.front = ActiveHinge(0.0)
    part2.left = ActiveHinge(np.pi / 2.0)
    part2.left.attachment = ActiveHinge(-np.pi / 2.0)
    part2.left.attachment.attachment = Brick(0.0)
    part2.left.attachment.attachment.right = ActiveHinge(np.pi / 2.0)
    part2.left.attachment.attachment.left = ActiveHinge(np.pi / 2.0)
    part2.left.attachment.attachment.left.attachment = ActiveHinge(0.0)

    body.finalize()
    return body


def penguin_v1() -> Body:
    """
    Get the penguin modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.right = Brick(0.0)
    body.core.right.left = ActiveHinge(np.pi / 2.0)
    body.core.right.left.attachment = ActiveHinge(-np.pi / 2.0)
    body.core.right.left.attachment.attachment = Brick(0.0)
    body.core.right.left.attachment.attachment.right = ActiveHinge(0.0)
    body.core.right.left.attachment.attachment.left = ActiveHinge(np.pi / 2.0)
    body.core.right.left.attachment.attachment.left.attachment = ActiveHinge(
        -np.pi / 2.0
    )
    body.core.right.left.attachment.attachment.left.attachment.attachment = ActiveHinge(
        np.pi / 2.0
    )
    body.core.right.left.attachment.attachment.left.attachment.attachment.attachment = (
        Brick(-np.pi / 2.0)
    )

    part2 = (
        body.core.right.left.attachment.attachment.left.attachment.attachment.attachment
    )

    part2.front = ActiveHinge(np.pi / 2.0)
    part2.front.attachment = Brick(-np.pi / 2.0)

    part2.right = ActiveHinge(0.0)
    part2.right.attachment = ActiveHinge(0.0)
    part2.right.attachment.attachment = ActiveHinge(np.pi / 2.0)
    part2.right.attachment.attachment.attachment = Brick(-np.pi / 2.0)

    part2.right.attachment.attachment.attachment.left = ActiveHinge(np.pi / 2.0)

    part2.right.attachment.attachment.attachment.right = Brick(0.0)
    part2.right.attachment.attachment.attachment.right.front = ActiveHinge(np.pi / 2.0)

    body.finalize()
    return body


def pentapod_v1() -> Body:
    """
    Get the pentapod modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.right = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment = ActiveHinge(0.0)
    body.core.right.attachment.attachment = ActiveHinge(0.0)
    body.core.right.attachment.attachment.attachment = ActiveHinge(-np.pi / 2.0)
    body.core.right.attachment.attachment.attachment.attachment = Brick(0.0)
    part2 = body.core.right.attachment.attachment.attachment.attachment

    part2.left = ActiveHinge(0.0)
    part2.front = ActiveHinge(np.pi / 2.0)
    part2.front.attachment = Brick(-np.pi / 2.0)
    part2.front.attachment.left = Brick(0.0)
    part2.front.attachment.right = ActiveHinge(0.0)
    part2.front.attachment.front = ActiveHinge(np.pi / 2.0)
    part2.front.attachment.front.attachment = Brick(-np.pi / 2.0)
    part2.front.attachment.front.attachment.left = ActiveHinge(0.0)
    part2.front.attachment.front.attachment.right = ActiveHinge(0.0)

    body.finalize()
    return body


def queen_v1() -> Body:
    """
    Get the queen modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.back = ActiveHinge(np.pi / 2.0)
    body.core.right = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment = ActiveHinge(0.0)
    body.core.right.attachment.attachment = ActiveHinge(-np.pi / 2.0)
    body.core.right.attachment.attachment.attachment = Brick(0.0)
    part2 = body.core.right.attachment.attachment.attachment

    part2.left = ActiveHinge(0.0)
    part2.right = Brick(0.0)
    part2.right.front = Brick(0.0)
    part2.right.front.left = ActiveHinge(0.0)
    part2.right.front.right = ActiveHinge(0.0)

    part2.right.right = Brick(0.0)
    part2.right.right.front = ActiveHinge(np.pi / 2.0)
    part2.right.right.front.attachment = ActiveHinge(0.0)

    body.finalize()
    return body


def squarish_v1() -> Body:
    """
    Get the squarish modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.back = ActiveHinge(0.0)
    body.core.back.attachment = Brick(0.0)
    body.core.back.attachment.front = ActiveHinge(0.0)
    body.core.back.attachment.left = ActiveHinge(np.pi / 2.0)
    body.core.back.attachment.left.attachment = Brick(-np.pi / 2.0)
    body.core.back.attachment.left.attachment.left = Brick(0.0)
    part2 = body.core.back.attachment.left.attachment.left

    part2.left = ActiveHinge(np.pi / 2.0)
    part2.front = ActiveHinge(0.0)
    part2.right = ActiveHinge(np.pi / 2.0)
    part2.right.attachment = Brick(-np.pi / 2.0)
    part2.right.attachment.left = Brick(0.0)
    part2.right.attachment.left.left = Brick(0.0)

    body.finalize()
    return body


def snake_v1() -> Body:
    """
    Get the snake modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.left = ActiveHinge(0.0)
    body.core.left.attachment = Brick(0.0)
    body.core.left.attachment.front = ActiveHinge(np.pi / 2.0)
    body.core.left.attachment.front.attachment = Brick(-np.pi / 2.0)
    body.core.left.attachment.front.attachment.front = ActiveHinge(0.0)
    body.core.left.attachment.front.attachment.front.attachment = Brick(0.0)
    body.core.left.attachment.front.attachment.front.attachment.front = ActiveHinge(
        np.pi / 2.0
    )
    body.core.left.attachment.front.attachment.front.attachment.front.attachment = (
        Brick(-np.pi / 2.0)
    )
    body.core.left.attachment.front.attachment.front.attachment.front.attachment.front = ActiveHinge(
        0.0
    )
    body.core.left.attachment.front.attachment.front.attachment.front.attachment.front.attachment = Brick(
        0.0
    )
    body.core.left.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front = ActiveHinge(
        np.pi / 2.0
    )
    body.core.left.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front.attachment = Brick(
        -np.pi / 2.0
    )
    body.core.left.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front = ActiveHinge(
        0.0
    )
    body.core.left.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front.attachment = Brick(
        0.0
    )
    body.core.left.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front = ActiveHinge(
        np.pi / 2.0
    )

    body.finalize()
    return body


def stingray_v1() -> Body:
    """
    Get the stingray modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.back = ActiveHinge(np.pi / 2.0)
    body.core.right = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment = ActiveHinge(-np.pi / 2.0)
    body.core.right.attachment.attachment = Brick(0.0)
    body.core.right.attachment.attachment.right = Brick(0.0)
    body.core.right.attachment.attachment.left = ActiveHinge(0.0)
    body.core.right.attachment.attachment.front = Brick(0.0)
    body.core.right.attachment.attachment.front.right = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment.attachment.front.front = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment.attachment.front.left = ActiveHinge(0.0)
    body.core.right.attachment.attachment.front.left.attachment = Brick(0.0)
    body.core.right.attachment.attachment.front.left.attachment.right = ActiveHinge(
        np.pi / 2.0
    )
    body.core.right.attachment.attachment.front.left.attachment.front = ActiveHinge(0.0)
    body.core.right.attachment.attachment.front.left.attachment.front.attachment = (
        Brick(0.0)
    )

    body.finalize()
    return body


def tinlicker_v1() -> Body:
    """
    Get the tinlicker modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.right = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment = ActiveHinge(0.0)
    body.core.right.attachment.attachment = ActiveHinge(0.0)
    body.core.right.attachment.attachment.attachment = ActiveHinge(-np.pi / 2.0)
    body.core.right.attachment.attachment.attachment.attachment = Brick(0.0)
    part2 = body.core.right.attachment.attachment.attachment.attachment

    part2.left = Brick(0.0)
    part2.left.front = ActiveHinge(np.pi / 2.0)
    part2.left.right = Brick(0.0)
    part2.left.right.left = Brick(0.0)
    part2.left.right.front = ActiveHinge(0.0)
    part2.left.right.front.attachment = Brick(0.0)
    part2.left.right.front.attachment.front = ActiveHinge(np.pi / 2.0)
    part2.left.right.front.attachment.right = Brick(0.0)
    part2.left.right.front.attachment.right.right = ActiveHinge(np.pi / 2.0)

    body.finalize()
    return body


def turtle_v1() -> Body:
    """
    Get the turtle modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.left = Brick(0.0)
    body.core.left.right = ActiveHinge(0.0)
    body.core.left.left = ActiveHinge(np.pi / 2.0)
    body.core.left.left.attachment = ActiveHinge(-np.pi / 2.0)
    body.core.left.left.attachment.attachment = Brick(0.0)

    body.core.left.left.attachment.attachment.front = Brick(0.0)
    body.core.left.left.attachment.attachment.left = ActiveHinge(np.pi / 2.0)
    body.core.left.left.attachment.attachment.right = ActiveHinge(0.0)
    body.core.left.left.attachment.attachment.right.attachment = Brick(0.0)
    part2 = body.core.left.left.attachment.attachment.right.attachment

    part2.left = ActiveHinge(np.pi / 2.0)
    part2.left.attachment = ActiveHinge(-np.pi / 2.0)
    part2.front = Brick(0.0)
    part2.right = ActiveHinge(0.0)
    part2.right.attachment = Brick(0.0)
    part2.right.attachment.right = ActiveHinge(0.0)
    part2.right.attachment.left = ActiveHinge(np.pi / 2.0)
    part2.right.attachment.left.attachment = ActiveHinge(-np.pi / 2.0)
    part2.right.attachment.left.attachment.attachment = ActiveHinge(0.0)
    part2.right.attachment.left.attachment.attachment.attachment = ActiveHinge(0.0)

    body.finalize()
    return body


def ww_v1() -> Body:
    """
    Get the ww modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.back = ActiveHinge(0.0)
    body.core.right = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment = ActiveHinge(0.0)
    body.core.right.attachment.attachment = ActiveHinge(-np.pi / 2.0)
    body.core.right.attachment.attachment.attachment = Brick(0.0)
    body.core.right.attachment.attachment.attachment.left = ActiveHinge(0.0)
    body.core.right.attachment.attachment.attachment.left.attachment = Brick(0.0)
    part2 = body.core.right.attachment.attachment.attachment.left.attachment

    part2.left = ActiveHinge(0.0)
    part2.front = Brick(0.0)
    part2.front.right = ActiveHinge(np.pi / 2.0)
    part2.front.right.attachment = Brick(-np.pi / 2.0)
    part2.front.right.attachment.left = ActiveHinge(np.pi / 2.0)
    part2.front.right.attachment.left.attachment = ActiveHinge(0.0)
    part2.front.right.attachment.left.attachment.attachment = ActiveHinge(-np.pi / 2.0)

    body.finalize()
    return body


def zappa_v1() -> Body:
    """
    Get the zappa modular robot.

    :returns: the robot.
    """
    body = Body(_PROPERTIES)

    body.core.back = ActiveHinge(0.0)
    body.core.right = ActiveHinge(np.pi / 2.0)
    body.core.right.attachment = ActiveHinge(0.0)
    body.core.right.attachment.attachment = ActiveHinge(0.0)
    body.core.right.attachment.attachment.attachment = ActiveHinge(-np.pi / 2.0)
    body.core.right.attachment.attachment.attachment.attachment = ActiveHinge(0.0)
    body.core.right.attachment.attachment.attachment.attachment.attachment = Brick(0.0)
    part2 = body.core.right.attachment.attachment.attachment.attachment.attachment

    part2.front = ActiveHinge(0.0)
    part2.front.attachment = ActiveHinge(0.0)
    part2.left = ActiveHinge(np.pi / 2.0)
    part2.left.attachment = Brick(-np.pi / 2.0)
    part2.left.attachment.left = ActiveHinge(0.0)
    part2.left.attachment.left.attachment = Brick(0.0)
    part2.left.attachment.front = ActiveHinge(0.0)

    body.finalize()
    return body
