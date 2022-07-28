"""Standard modular robots."""

from typing import List

import numpy as np
from revolve2.core.modular_robot import ActiveHinge, Body, Brick


def all() -> List[Body]:
    """
    Get a list of all standard module robots.

    :returns: The list of robots.
    """
    return [
        babya(),
        babyb(),
        blokky(),
        garrix(),
        gecko(),
        insect(),
        linkin(),
        longleg(),
        penguin(),
        pentapod(),
        queen(),
        salamander(),
        squarish(),
        snake(),
        spider(),
        stingray(),
        tinlicker(),
        turtle(),
        ww(),
        zappa(),
        ant(),
        park(),
    ]


def get(name: str) -> Body:
    """
    Get a robot by name.

    :param name: The name of the robot to get.
    :returns: The robot with that name.
    :raises ValueError: When a robot with that name does not exist.
    """
    if name == "spider":
        return spider()
    elif name == "gecko":
        return gecko()
    elif name == "babya":
        return babya()
    elif name == "ant":
        return ant()
    elif name == "salamander":
        return salamander()
    elif name == "blokky":
        return blokky()
    elif name == "park":
        return park()
    elif name == "babyb":
        return babyb()
    elif name == "garrix":
        return garrix()
    elif name == "insect":
        return insect()
    elif name == "linkin":
        return linkin()
    elif name == "longleg":
        return longleg()
    elif name == "penguin":
        return penguin()
    elif name == "pentapod":
        return pentapod()
    elif name == "queen":
        return queen()
    elif name == "squarish":
        return squarish()
    elif name == "snake":
        return snake()
    elif name == "stingray":
        return stingray()
    elif name == "tinlicker":
        return tinlicker()
    elif name == "turtle":
        return turtle()
    elif name == "ww":
        return ww()
    elif name == "zappa":
        return zappa()
    else:
        raise ValueError(f"Robot does not exist: {name}")


def spider() -> Body:
    """
    Get the spider modular robot.

    :returns: the robot.
    """
    body = Body()

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


def gecko() -> Body:
    """
    Get the gecko modular robot.

    :returns: the robot.
    """
    body = Body()

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


def babya() -> Body:
    """
    Get the babya modular robot.

    :returns: the robot.
    """
    body = Body()

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


def ant() -> Body:
    """
    Get the ant modular robot.

    :returns: the robot.
    """
    body = Body()

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


def salamander() -> Body:
    """
    Get the salamander modular robot.

    :returns: the robot.
    """
    body = Body()

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


def blokky() -> Body:
    """
    Get the blokky modular robot.

    :returns: the robot.
    """
    body = Body()

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


def park() -> Body:
    """
    Get the park modular robot.

    :returns: the robot.
    """
    body = Body()

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


def babyb() -> Body:
    """
    Get the babyb modular robot.

    :returns: the robot.
    """
    body = Body()

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


def garrix() -> Body:
    """
    Get the garrix modular robot.

    :returns: the robot.
    """
    body = Body()

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


def insect() -> Body:
    """
    Get the insect modular robot.

    :returns: the robot.
    """
    body = Body()

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


def linkin() -> Body:
    """
    Get the linkin modular robot.

    :returns: the robot.
    """
    body = Body()

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


def longleg() -> Body:
    """
    Get the longleg modular robot.

    :returns: the robot.
    """
    body = Body()

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


def penguin() -> Body:
    """
    Get the penguin modular robot.

    :returns: the robot.
    """
    body = Body()

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


def pentapod() -> Body:
    """
    Get the pentapod modular robot.

    :returns: the robot.
    """
    body = Body()

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


def queen() -> Body:
    """
    Get the queen modular robot.

    :returns: the robot.
    """
    body = Body()

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


def squarish() -> Body:
    """
    Get the squarish modular robot.

    :returns: the robot.
    """
    body = Body()

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


def snake() -> Body:
    """
    Get the snake modular robot.

    :returns: the robot.
    """
    body = Body()

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


def stingray() -> Body:
    """
    Get the stingray modular robot.

    :returns: the robot.
    """
    body = Body()

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


def tinlicker() -> Body:
    """
    Get the tinlicker modular robot.

    :returns: the robot.
    """
    body = Body()

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


def turtle() -> Body:
    """
    Get the turtle modular robot.

    :returns: the robot.
    """
    body = Body()

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


def ww() -> Body:
    """
    Get the ww modular robot.

    :returns: the robot.
    """
    body = Body()

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


def zappa() -> Body:
    """
    Get the zappa modular robot.

    :returns: the robot.
    """
    body = Body()

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
