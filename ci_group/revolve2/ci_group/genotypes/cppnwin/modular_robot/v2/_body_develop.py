import math
from dataclasses import dataclass
from queue import Queue
from typing import Any

import multineat
import numpy as np
from numpy.typing import NDArray
from pyrr import Vector3

from revolve2.modular_robot.body import Module
from revolve2.modular_robot.body.v2 import ActiveHingeV2, BodyV2, BrickV2, CoreV2


@dataclass
class __Module:
    position: Vector3
    forward: Vector3
    up: Vector3
    chain_length: int
    module_reference: Module


def develop(
    genotype: multineat.Genome,
) -> BodyV2:
    """
    Develop a CPPNWIN genotype into a modular robot body.

    It is important that the genotype was created using a compatible function.

    :param genotype: The genotype to create the body from.
    :returns: The create body.
    """
    max_parts = 10

    body_net = multineat.NeuralNetwork()
    genotype.BuildPhenotype(body_net)

    to_explore: Queue[__Module] = Queue()
    grid = np.zeros(shape=(max_parts, max_parts, max_parts), dtype=np.uint8)

    outputs = __evaluate_cppn(body_net, Vector3([0, 0, 0]), 0)

    attachment_positions = list(map(__get_position_index, outputs[5:]))
    body = BodyV2()

    to_explore.put(
        __Module(
            Vector3([0, 0, 0]), Vector3([0, -1, 0]), Vector3([0, 0, 1]), 0, body.core
        )
    )
    grid[0, 0, 0] = 1
    part_count = 1

    while not to_explore.empty():
        module = to_explore.get()

        for index in module.module_reference.attachment_points.keys():
            if part_count < max_parts:
                child = __add_child(body_net, module, index, grid, attachment_positions)
                if child is not None:
                    to_explore.put(child)
                    part_count += 1
    return body


def __evaluate_cppn(
    body_net: multineat.NeuralNetwork,
    position: Vector3,
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
    types = [None, BrickV2, ActiveHingeV2]
    module_type = types[type_probs.index(min(type_probs))]

    # get rotation from output probabilities
    rotation_probs = [outputs[3], outputs[4]]
    rotation = rotation_probs.index(min(rotation_probs))

    # get attachment position
    return (module_type, rotation)


def __add_child(
    body_net: multineat.NeuralNetwork,
    module: __Module,
    child_index: int,
    grid: NDArray[np.uint8],
    attachment_positions: list[int] | None = None,
) -> __Module | None:
    forward = __rotate(module.forward, module.up, child_index)
    position = module.position + forward
    chain_length = module.chain_length + 1

    # if grid cell is occupied, don't make a child
    # else, set cell as occupied
    if grid[tuple(position)] > 0:
        return None
    grid[tuple(position)] += 1

    child_type, orientation = __evaluate_cppn(body_net, position, chain_length)
    if child_type is None:
        return None
    up = __rotate(module.up, forward, orientation)

    child = child_type(orientation * (math.pi / 2.0))

    global_index = child_index
    if attachment_positions is not None and isinstance(module.module_reference, CoreV2):
        global_index = module.module_reference.index_from_face_and_attachment(
            child_index, attachment_positions[child_index]
        )

    module.module_reference.set_child(child, global_index)

    return __Module(
        position,
        forward,
        up,
        chain_length,
        child,
    )


def __rotate(a: Vector3, b: Vector3, angle: int) -> Vector3:
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
    match angle:
        case 0:
            cosangle = 1
            sinangle = 0
        case 1:
            cosangle = 0
            sinangle = 1
        case 2:
            cosangle = -1
            sinangle = 0
        case 3:
            cosangle = 0
            sinangle = -1

    return (a * cosangle + b.cross(a) * sinangle) + b * (b.dot(a) * (1 - cosangle))


def __get_position_index(x: float) -> int:
    return max(1, int(abs(x) * 10 - 1e-3))
