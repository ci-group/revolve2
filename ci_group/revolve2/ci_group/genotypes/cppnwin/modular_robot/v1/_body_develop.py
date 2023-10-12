import math
from dataclasses import dataclass
from queue import Queue
from typing import Any

import multineat

from revolve2.modular_robot.body import Module
from revolve2.modular_robot.body.v1 import ActiveHingeV1, BodyV1, BrickV1, CoreV1


@dataclass
class __Module:
    position: tuple[int, int, int]
    forward: tuple[int, int, int]
    up: tuple[int, int, int]
    chain_length: int
    module_reference: Module


def develop(
    genotype: multineat.Genome,
) -> BodyV1:
    """
    Develop a CPPNWIN genotype into a modular robot body.

    It is important that the genotype was created using a compatible function.

    :param genotype: The genotype to create the body from.
    :returns: The created body.
    :raises RuntimeError: In case a module is encountered that is not supported.
    """
    max_parts = 10

    body_net = multineat.NeuralNetwork()
    genotype.BuildPhenotype(body_net)

    to_explore: Queue[__Module] = Queue()
    grid: set[tuple[int, int, int]] = set()

    body = BodyV1()

    to_explore.put(__Module((0, 0, 0), (0, -1, 0), (0, 0, 1), 0, body.core))
    grid.add((0, 0, 0))
    part_count = 1

    while not to_explore.empty():
        module = to_explore.get()

        children: list[tuple[int, int]] = []  # child index, rotation

        if isinstance(module.module_reference, CoreV1):
            children.append((CoreV1.FRONT, 0))
            children.append((CoreV1.LEFT, 1))
            children.append((CoreV1.BACK, 2))
            children.append((CoreV1.RIGHT, 3))
        elif isinstance(module.module_reference, BrickV1):
            children.append((BrickV1.FRONT, 0))
            children.append((BrickV1.LEFT, 1))
            children.append((BrickV1.RIGHT, 3))
        elif isinstance(module.module_reference, ActiveHingeV1):
            children.append((ActiveHingeV1.ATTACHMENT, 0))
        else:  # Should actually never arrive here but just checking module type to be sure
            raise RuntimeError()

        for index, rotation in children:
            if part_count < max_parts:
                child = __add_child(body_net, module, index, rotation, grid)
                if child is not None:
                    to_explore.put(child)
                    part_count += 1

    return body


def __evaluate_cppn(
    body_net: multineat.NeuralNetwork,
    position: tuple[int, int, int],
    chain_length: int,
) -> tuple[Any, int]:
    """
    Get module type and orientation from a multineat CPPN network.

    :param body_net: The CPPN network.
    :param position: Position of the module.
    :param chain_length: Tree distance of the module from the core.
    :returns: (module type, orientation)
    """
    body_net.Input(
        [1.0, position[0], position[1], position[2], chain_length]
    )  # 1.0 is the bias input
    body_net.ActivateAllLayers()
    outputs = body_net.Output()

    # get module type from output probabilities
    type_probs = [outputs[0], outputs[1], outputs[2]]
    types = [None, BrickV1, ActiveHingeV1]
    module_type = types[type_probs.index(min(type_probs))]

    # get rotation from output probabilities
    rotation_probs = [outputs[3], outputs[4]]
    rotation = rotation_probs.index(min(rotation_probs))

    return (module_type, rotation)


def __add_child(
    body_net: multineat.NeuralNetwork,
    module: __Module,
    child_index: int,
    rotation: int,
    grid: set[tuple[int, int, int]],
) -> __Module | None:
    forward = __rotate(module.forward, module.up, rotation)
    position = __add(module.position, forward)
    chain_length = module.chain_length + 1

    # if grid cell is occupied, don't make a child
    # else, set cell as occupied
    if position in grid:
        return None
    else:
        grid.add(position)

    child_type, orientation = __evaluate_cppn(body_net, position, chain_length)
    if child_type is None:
        return None
    up = __rotate(module.up, forward, orientation)

    child = child_type(orientation * (math.pi / 2.0))
    module.module_reference.set_child(child, child_index)

    return __Module(
        position,
        forward,
        up,
        chain_length,
        child,
    )


def __add(a: tuple[int, int, int], b: tuple[int, int, int]) -> tuple[int, int, int]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def __timesscalar(a: tuple[int, int, int], scalar: int) -> tuple[int, int, int]:
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def __cross(a: tuple[int, int, int], b: tuple[int, int, int]) -> tuple[int, int, int]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def __dot(a: tuple[int, int, int], b: tuple[int, int, int]) -> int:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def __rotate(
    a: tuple[int, int, int], b: tuple[int, int, int], angle: int
) -> tuple[int, int, int]:
    """
    Rotates vector a a given angle around b.

    Angle from [0,1,2,3].
    90 degrees each.

    :param a: Vector a.
    :param b: Vector b.
    :param angle: The angle to rotate.
    :returns: A copy of a, rotated.
    """
    cosangle: int
    sinangle: int
    if angle == 0:
        cosangle = 1
        sinangle = 0
    elif angle == 1:
        cosangle = 0
        sinangle = 1
    elif angle == 2:
        cosangle = -1
        sinangle = 0
    else:
        cosangle = 0
        sinangle = -1

    return __add(
        __add(
            __timesscalar(a, cosangle),
            __timesscalar(__cross(b, a), sinangle),
        ),
        __timesscalar(b, __dot(b, a) * (1 - cosangle)),
    )
