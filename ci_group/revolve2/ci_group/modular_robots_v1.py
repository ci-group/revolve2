"""Standard modular robots."""

import numpy as np

from revolve2.modular_robot.body.v1 import ActiveHingeV1, BodyV1, BrickV1


def all() -> list[BodyV1]:
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


def spider_v1() -> BodyV1:
    """
    Get the spider modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.left = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.left.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.left.attachment.front = ActiveHingeV1(0.0)
    body.core_v1.left.attachment.front.attachment = BrickV1(0.0)

    body.core_v1.right = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.right.attachment.front = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.front.attachment = BrickV1(0.0)

    body.core_v1.front = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.front.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.front.attachment.front = ActiveHingeV1(0.0)
    body.core_v1.front.attachment.front.attachment = BrickV1(0.0)

    body.core_v1.back = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.back.attachment.front = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.front.attachment = BrickV1(0.0)

    return body


def gecko_v1() -> BodyV1:
    """
    Get the gecko modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.left = ActiveHingeV1(0.0)
    body.core_v1.left.attachment = BrickV1(0.0)

    body.core_v1.right = ActiveHingeV1(0.0)
    body.core_v1.right.attachment = BrickV1(0.0)

    body.core_v1.back = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.back.attachment.front = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back.attachment.front.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.back.attachment.front.attachment.left = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.front.attachment.left.attachment = BrickV1(0.0)
    body.core_v1.back.attachment.front.attachment.right = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.front.attachment.right.attachment = BrickV1(0.0)

    return body


def babya_v1() -> BodyV1:
    """
    Get the babya modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.left = ActiveHingeV1(0.0)
    body.core_v1.left.attachment = BrickV1(0.0)

    body.core_v1.right = ActiveHingeV1(0.0)
    body.core_v1.right.attachment = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.right.attachment.attachment.front = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment.front.attachment = BrickV1(0.0)

    body.core_v1.back = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.back.attachment.front = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back.attachment.front.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.back.attachment.front.attachment.left = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.front.attachment.left.attachment = BrickV1(0.0)
    body.core_v1.back.attachment.front.attachment.right = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.front.attachment.right.attachment = BrickV1(0.0)

    return body


def ant_v1() -> BodyV1:
    """
    Get the ant modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.left = ActiveHingeV1(0.0)
    body.core_v1.left.attachment = BrickV1(0.0)

    body.core_v1.right = ActiveHingeV1(0.0)
    body.core_v1.right.attachment = BrickV1(0.0)

    body.core_v1.back = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.back.attachment.left = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.left.attachment = BrickV1(0.0)
    body.core_v1.back.attachment.right = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.right.attachment = BrickV1(0.0)

    body.core_v1.back.attachment.front = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back.attachment.front.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.back.attachment.front.attachment.left = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.front.attachment.left.attachment = BrickV1(0.0)
    body.core_v1.back.attachment.front.attachment.right = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.front.attachment.right.attachment = BrickV1(0.0)

    return body


def salamander_v1() -> BodyV1:
    """
    Get the salamander modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.left = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.left.attachment = ActiveHingeV1(-np.pi / 2.0)

    body.core_v1.right = ActiveHingeV1(0.0)

    body.core_v1.back = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.back.attachment.left = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.front = BrickV1(0.0)
    body.core_v1.back.attachment.front.left = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.front.front = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back.attachment.front.front.attachment = BrickV1(-np.pi / 2.0)

    body.core_v1.back.attachment.front.front.attachment.left = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.front.front.attachment.left.attachment = BrickV1(0.0)
    body.core_v1.back.attachment.front.front.attachment.left.attachment.left = BrickV1(
        0.0
    )
    body.core_v1.back.attachment.front.front.attachment.left.attachment.front = (
        ActiveHingeV1(np.pi / 2.0)
    )
    body.core_v1.back.attachment.front.front.attachment.left.attachment.front.attachment = ActiveHingeV1(
        -np.pi / 2.0
    )

    body.core_v1.back.attachment.front.front.attachment.front = BrickV1(0.0)
    body.core_v1.back.attachment.front.front.attachment.front.left = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.front.front.attachment.front.front = BrickV1(0.0)
    body.core_v1.back.attachment.front.front.attachment.front.front.left = (
        ActiveHingeV1(0.0)
    )
    body.core_v1.back.attachment.front.front.attachment.front.front.front = BrickV1(0.0)
    body.core_v1.back.attachment.front.front.attachment.front.front.front.front = (
        ActiveHingeV1(np.pi / 2.0)
    )
    body.core_v1.back.attachment.front.front.attachment.front.front.front.front.attachment = BrickV1(
        -np.pi / 2.0
    )
    body.core_v1.back.attachment.front.front.attachment.front.front.front.front.attachment.left = BrickV1(
        0.0
    )
    body.core_v1.back.attachment.front.front.attachment.front.front.front.front.attachment.front = ActiveHingeV1(
        np.pi / 2.0
    )

    return body


def blokky_v1() -> BodyV1:
    """
    Get the blokky modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.left = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back = BrickV1(0.0)
    body.core_v1.back.right = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back.front = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back.front.attachment = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.back.front.attachment.attachment = BrickV1(0.0)
    body.core_v1.back.front.attachment.attachment.front = BrickV1(0.0)
    body.core_v1.back.front.attachment.attachment.front.right = BrickV1(0.0)
    body.core_v1.back.front.attachment.attachment.front.right.left = BrickV1(0.0)
    body.core_v1.back.front.attachment.attachment.front.right.front = BrickV1(0.0)
    body.core_v1.back.front.attachment.attachment.right = BrickV1(0.0)
    body.core_v1.back.front.attachment.attachment.right.front = BrickV1(0.0)
    body.core_v1.back.front.attachment.attachment.right.front.right = BrickV1(0.0)
    body.core_v1.back.front.attachment.attachment.right.front.front = ActiveHingeV1(0.0)

    return body


def park_v1() -> BodyV1:
    """
    Get the park modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.back = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back.attachment = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.back.attachment.attachment = BrickV1(0.0)
    body.core_v1.back.attachment.attachment.right = BrickV1(0.0)
    body.core_v1.back.attachment.attachment.left = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.attachment.front = BrickV1(0.0)
    body.core_v1.back.attachment.attachment.front.right = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.back.attachment.attachment.front.front = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.back.attachment.attachment.front.left = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.attachment.front.left.attachment = BrickV1(0.0)
    body.core_v1.back.attachment.attachment.front.left.attachment.right = ActiveHingeV1(
        -np.pi / 2.0
    )
    body.core_v1.back.attachment.attachment.front.left.attachment.left = BrickV1(0.0)
    body.core_v1.back.attachment.attachment.front.left.attachment.front = ActiveHingeV1(
        0.0
    )
    body.core_v1.back.attachment.attachment.front.left.attachment.front.attachment = (
        BrickV1(0.0)
    )

    return body


def babyb_v1() -> BodyV1:
    """
    Get the babyb modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.left = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.left.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.left.attachment.front = ActiveHingeV1(0.0)
    body.core_v1.left.attachment.front.attachment = BrickV1(0.0)
    body.core_v1.left.attachment.front.attachment.front = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.left.attachment.front.attachment.front.attachment = BrickV1(0.0)

    body.core_v1.right = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.right.attachment.front = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.front.attachment = BrickV1(0.0)
    body.core_v1.right.attachment.front.attachment.front = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment.front.attachment.front.attachment = BrickV1(0.0)

    body.core_v1.front = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.front.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.front.attachment.front = ActiveHingeV1(0.0)
    body.core_v1.front.attachment.front.attachment = BrickV1(0.0)
    body.core_v1.front.attachment.front.attachment.front = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.front.attachment.front.attachment.front.attachment = BrickV1(0.0)

    body.core_v1.back = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back.attachment = BrickV1(-np.pi / 2.0)

    return body


def garrix_v1() -> BodyV1:
    """
    Get the garrix modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.front = ActiveHingeV1(np.pi / 2.0)

    body.core_v1.left = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.left.attachment = ActiveHingeV1(0.0)
    body.core_v1.left.attachment.attachment = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.left.attachment.attachment.attachment = BrickV1(0.0)
    body.core_v1.left.attachment.attachment.attachment.front = BrickV1(0.0)
    body.core_v1.left.attachment.attachment.attachment.left = ActiveHingeV1(0.0)

    part2 = BrickV1(0.0)
    part2.right = ActiveHingeV1(np.pi / 2.0)
    part2.front = ActiveHingeV1(np.pi / 2.0)
    part2.left = ActiveHingeV1(0.0)
    part2.left.attachment = ActiveHingeV1(np.pi / 2.0)
    part2.left.attachment.attachment = ActiveHingeV1(-np.pi / 2.0)
    part2.left.attachment.attachment.attachment = BrickV1(0.0)

    body.core_v1.left.attachment.attachment.attachment.left.attachment = part2

    return body


def insect_v1() -> BodyV1:
    """
    Get the insect modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.right = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.right.attachment.attachment = BrickV1(0.0)
    body.core_v1.right.attachment.attachment.right = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment.front = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment.attachment.left = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment.attachment.left.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.right.attachment.attachment.left.attachment.front = ActiveHingeV1(
        np.pi / 2.0
    )
    body.core_v1.right.attachment.attachment.left.attachment.right = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment.left.attachment.right.attachment = (
        ActiveHingeV1(0.0)
    )
    body.core_v1.right.attachment.attachment.left.attachment.right.attachment.attachment = ActiveHingeV1(
        np.pi / 2.0
    )

    return body


def linkin_v1() -> BodyV1:
    """
    Get the linkin modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.back = ActiveHingeV1(0.0)

    body.core_v1.right = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment.attachment = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.right.attachment.attachment.attachment.attachment = BrickV1(0.0)

    part2 = body.core_v1.right.attachment.attachment.attachment.attachment
    part2.front = BrickV1(0.0)

    part2.left = ActiveHingeV1(0.0)
    part2.left.attachment = ActiveHingeV1(0.0)

    part2.right = ActiveHingeV1(np.pi / 2.0)
    part2.right.attachment = ActiveHingeV1(-np.pi / 2.0)
    part2.right.attachment.attachment = ActiveHingeV1(0.0)
    part2.right.attachment.attachment.attachment = ActiveHingeV1(np.pi / 2.0)
    part2.right.attachment.attachment.attachment.attachment = ActiveHingeV1(0.0)

    return body


def longleg_v1() -> BodyV1:
    """
    Get the longleg modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.left = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.left.attachment = ActiveHingeV1(0.0)
    body.core_v1.left.attachment.attachment = ActiveHingeV1(0.0)
    body.core_v1.left.attachment.attachment.attachment = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.left.attachment.attachment.attachment.attachment = ActiveHingeV1(0.0)
    body.core_v1.left.attachment.attachment.attachment.attachment.attachment = BrickV1(
        0.0
    )

    part2 = body.core_v1.left.attachment.attachment.attachment.attachment.attachment
    part2.right = ActiveHingeV1(0.0)
    part2.front = ActiveHingeV1(0.0)
    part2.left = ActiveHingeV1(np.pi / 2.0)
    part2.left.attachment = ActiveHingeV1(-np.pi / 2.0)
    part2.left.attachment.attachment = BrickV1(0.0)
    part2.left.attachment.attachment.right = ActiveHingeV1(np.pi / 2.0)
    part2.left.attachment.attachment.left = ActiveHingeV1(np.pi / 2.0)
    part2.left.attachment.attachment.left.attachment = ActiveHingeV1(0.0)

    return body


def penguin_v1() -> BodyV1:
    """
    Get the penguin modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.right = BrickV1(0.0)
    body.core_v1.right.left = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.left.attachment = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.right.left.attachment.attachment = BrickV1(0.0)
    body.core_v1.right.left.attachment.attachment.right = ActiveHingeV1(0.0)
    body.core_v1.right.left.attachment.attachment.left = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.left.attachment.attachment.left.attachment = ActiveHingeV1(
        -np.pi / 2.0
    )
    body.core_v1.right.left.attachment.attachment.left.attachment.attachment = (
        ActiveHingeV1(np.pi / 2.0)
    )
    body.core_v1.right.left.attachment.attachment.left.attachment.attachment.attachment = BrickV1(
        -np.pi / 2.0
    )

    part2 = (
        body.core_v1.right.left.attachment.attachment.left.attachment.attachment.attachment
    )

    part2.front = ActiveHingeV1(np.pi / 2.0)
    part2.front.attachment = BrickV1(-np.pi / 2.0)

    part2.right = ActiveHingeV1(0.0)
    part2.right.attachment = ActiveHingeV1(0.0)
    part2.right.attachment.attachment = ActiveHingeV1(np.pi / 2.0)
    part2.right.attachment.attachment.attachment = BrickV1(-np.pi / 2.0)

    part2.right.attachment.attachment.attachment.left = ActiveHingeV1(np.pi / 2.0)

    part2.right.attachment.attachment.attachment.right = BrickV1(0.0)
    part2.right.attachment.attachment.attachment.right.front = ActiveHingeV1(
        np.pi / 2.0
    )

    return body


def pentapod_v1() -> BodyV1:
    """
    Get the pentapod modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.right = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment.attachment = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.right.attachment.attachment.attachment.attachment = BrickV1(0.0)
    part2 = body.core_v1.right.attachment.attachment.attachment.attachment

    part2.left = ActiveHingeV1(0.0)
    part2.front = ActiveHingeV1(np.pi / 2.0)
    part2.front.attachment = BrickV1(-np.pi / 2.0)
    part2.front.attachment.left = BrickV1(0.0)
    part2.front.attachment.right = ActiveHingeV1(0.0)
    part2.front.attachment.front = ActiveHingeV1(np.pi / 2.0)
    part2.front.attachment.front.attachment = BrickV1(-np.pi / 2.0)
    part2.front.attachment.front.attachment.left = ActiveHingeV1(0.0)
    part2.front.attachment.front.attachment.right = ActiveHingeV1(0.0)

    return body


def queen_v1() -> BodyV1:
    """
    Get the queen modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.back = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.right.attachment.attachment.attachment = BrickV1(0.0)
    part2 = body.core_v1.right.attachment.attachment.attachment

    part2.left = ActiveHingeV1(0.0)
    part2.right = BrickV1(0.0)
    part2.right.front = BrickV1(0.0)
    part2.right.front.left = ActiveHingeV1(0.0)
    part2.right.front.right = ActiveHingeV1(0.0)

    part2.right.right = BrickV1(0.0)
    part2.right.right.front = ActiveHingeV1(np.pi / 2.0)
    part2.right.right.front.attachment = ActiveHingeV1(0.0)

    return body


def squarish_v1() -> BodyV1:
    """
    Get the squarish modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.back = ActiveHingeV1(0.0)
    body.core_v1.back.attachment = BrickV1(0.0)
    body.core_v1.back.attachment.front = ActiveHingeV1(0.0)
    body.core_v1.back.attachment.left = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.back.attachment.left.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.back.attachment.left.attachment.left = BrickV1(0.0)
    part2 = body.core_v1.back.attachment.left.attachment.left

    part2.left = ActiveHingeV1(np.pi / 2.0)
    part2.front = ActiveHingeV1(0.0)
    part2.right = ActiveHingeV1(np.pi / 2.0)
    part2.right.attachment = BrickV1(-np.pi / 2.0)
    part2.right.attachment.left = BrickV1(0.0)
    part2.right.attachment.left.left = BrickV1(0.0)

    return body


def snake_v1() -> BodyV1:
    """
    Get the snake modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.left = ActiveHingeV1(0.0)
    body.core_v1.left.attachment = BrickV1(0.0)
    body.core_v1.left.attachment.front = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.left.attachment.front.attachment = BrickV1(-np.pi / 2.0)
    body.core_v1.left.attachment.front.attachment.front = ActiveHingeV1(0.0)
    body.core_v1.left.attachment.front.attachment.front.attachment = BrickV1(0.0)
    body.core_v1.left.attachment.front.attachment.front.attachment.front = (
        ActiveHingeV1(np.pi / 2.0)
    )
    body.core_v1.left.attachment.front.attachment.front.attachment.front.attachment = (
        BrickV1(-np.pi / 2.0)
    )
    body.core_v1.left.attachment.front.attachment.front.attachment.front.attachment.front = ActiveHingeV1(
        0.0
    )
    body.core_v1.left.attachment.front.attachment.front.attachment.front.attachment.front.attachment = BrickV1(
        0.0
    )
    body.core_v1.left.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front = ActiveHingeV1(
        np.pi / 2.0
    )
    body.core_v1.left.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front.attachment = BrickV1(
        -np.pi / 2.0
    )
    body.core_v1.left.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front = ActiveHingeV1(
        0.0
    )
    body.core_v1.left.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front.attachment = BrickV1(
        0.0
    )
    body.core_v1.left.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front.attachment.front = ActiveHingeV1(
        np.pi / 2.0
    )

    return body


def stingray_v1() -> BodyV1:
    """
    Get the stingray modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.back = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.right.attachment.attachment = BrickV1(0.0)
    body.core_v1.right.attachment.attachment.right = BrickV1(0.0)
    body.core_v1.right.attachment.attachment.left = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment.front = BrickV1(0.0)
    body.core_v1.right.attachment.attachment.front.right = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment.attachment.front.front = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment.attachment.front.left = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment.front.left.attachment = BrickV1(0.0)
    body.core_v1.right.attachment.attachment.front.left.attachment.right = (
        ActiveHingeV1(np.pi / 2.0)
    )
    body.core_v1.right.attachment.attachment.front.left.attachment.front = (
        ActiveHingeV1(0.0)
    )
    body.core_v1.right.attachment.attachment.front.left.attachment.front.attachment = (
        BrickV1(0.0)
    )

    return body


def tinlicker_v1() -> BodyV1:
    """
    Get the tinlicker modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.right = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment.attachment = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.right.attachment.attachment.attachment.attachment = BrickV1(0.0)
    part2 = body.core_v1.right.attachment.attachment.attachment.attachment

    part2.left = BrickV1(0.0)
    part2.left.front = ActiveHingeV1(np.pi / 2.0)
    part2.left.right = BrickV1(0.0)
    part2.left.right.left = BrickV1(0.0)
    part2.left.right.front = ActiveHingeV1(0.0)
    part2.left.right.front.attachment = BrickV1(0.0)
    part2.left.right.front.attachment.front = ActiveHingeV1(np.pi / 2.0)
    part2.left.right.front.attachment.right = BrickV1(0.0)
    part2.left.right.front.attachment.right.right = ActiveHingeV1(np.pi / 2.0)

    return body


def turtle_v1() -> BodyV1:
    """
    Get the turtle modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.left = BrickV1(0.0)
    body.core_v1.left.right = ActiveHingeV1(0.0)
    body.core_v1.left.left = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.left.left.attachment = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.left.left.attachment.attachment = BrickV1(0.0)

    body.core_v1.left.left.attachment.attachment.front = BrickV1(0.0)
    body.core_v1.left.left.attachment.attachment.left = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.left.left.attachment.attachment.right = ActiveHingeV1(0.0)
    body.core_v1.left.left.attachment.attachment.right.attachment = BrickV1(0.0)
    part2 = body.core_v1.left.left.attachment.attachment.right.attachment

    part2.left = ActiveHingeV1(np.pi / 2.0)
    part2.left.attachment = ActiveHingeV1(-np.pi / 2.0)
    part2.front = BrickV1(0.0)
    part2.right = ActiveHingeV1(0.0)
    part2.right.attachment = BrickV1(0.0)
    part2.right.attachment.right = ActiveHingeV1(0.0)
    part2.right.attachment.left = ActiveHingeV1(np.pi / 2.0)
    part2.right.attachment.left.attachment = ActiveHingeV1(-np.pi / 2.0)
    part2.right.attachment.left.attachment.attachment = ActiveHingeV1(0.0)
    part2.right.attachment.left.attachment.attachment.attachment = ActiveHingeV1(0.0)

    return body


def ww_v1() -> BodyV1:
    """
    Get the ww modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.back = ActiveHingeV1(0.0)
    body.core_v1.right = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.right.attachment.attachment.attachment = BrickV1(0.0)
    body.core_v1.right.attachment.attachment.attachment.left = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment.attachment.left.attachment = BrickV1(0.0)
    part2 = body.core_v1.right.attachment.attachment.attachment.left.attachment

    part2.left = ActiveHingeV1(0.0)
    part2.front = BrickV1(0.0)
    part2.front.right = ActiveHingeV1(np.pi / 2.0)
    part2.front.right.attachment = BrickV1(-np.pi / 2.0)
    part2.front.right.attachment.left = ActiveHingeV1(np.pi / 2.0)
    part2.front.right.attachment.left.attachment = ActiveHingeV1(0.0)
    part2.front.right.attachment.left.attachment.attachment = ActiveHingeV1(
        -np.pi / 2.0
    )

    return body


def zappa_v1() -> BodyV1:
    """
    Get the zappa modular robot.

    :returns: the robot.
    """
    body = BodyV1()

    body.core_v1.back = ActiveHingeV1(0.0)
    body.core_v1.right = ActiveHingeV1(np.pi / 2.0)
    body.core_v1.right.attachment = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment.attachment = ActiveHingeV1(-np.pi / 2.0)
    body.core_v1.right.attachment.attachment.attachment.attachment = ActiveHingeV1(0.0)
    body.core_v1.right.attachment.attachment.attachment.attachment.attachment = BrickV1(
        0.0
    )
    part2 = body.core_v1.right.attachment.attachment.attachment.attachment.attachment

    part2.front = ActiveHingeV1(0.0)
    part2.front.attachment = ActiveHingeV1(0.0)
    part2.left = ActiveHingeV1(np.pi / 2.0)
    part2.left.attachment = BrickV1(-np.pi / 2.0)
    part2.left.attachment.left = ActiveHingeV1(0.0)
    part2.left.attachment.left.attachment = BrickV1(0.0)
    part2.left.attachment.front = ActiveHingeV1(0.0)

    return body


def get(name: str) -> BodyV1:
    """
    Get a robot by name.

    :param name: The name of the robot to get.
    :returns: The robot with that name.
    :raises ValueError: When a robot with that name does not exist.
    """
    match name:
        case "gecko":
            return gecko_v1()
        case "spider":
            return spider_v1()
        case "babya":
            return babya_v1()
        case "ant":
            return ant_v1()
        case "salamander":
            return salamander_v1()
        case "blokky":
            return blokky_v1()
        case "park":
            return park_v1()
        case "babyb":
            return babyb_v1()
        case "garrix":
            return garrix_v1()
        case "insect":
            return insect_v1()
        case "linkin":
            return linkin_v1()
        case "longleg":
            return longleg_v1()
        case "penguin":
            return penguin_v1()
        case "pentapod":
            return pentapod_v1()
        case "queen":
            return queen_v1()
        case "squarish":
            return squarish_v1()
        case "snake":
            return snake_v1()
        case "stingray":
            return stingray_v1()
        case "tinlicker":
            return tinlicker_v1()
        case "turtle":
            return turtle_v1()
        case "ww":
            return ww_v1()
        case "zappa":
            return zappa_v1()
        case _:
            raise ValueError(f"Robot does not exist: {name}")
