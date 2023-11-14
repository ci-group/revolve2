from dataclasses import dataclass
from queue import Queue
from typing import Any

import multineat
import numpy as np
from numpy.typing import NDArray
from pyrr import Quaternion, Vector3

from revolve2.modular_robot.body import Module
from revolve2.modular_robot.body.v1 import ActiveHingeV1, BodyV1, BrickV1


@dataclass
class __Module:
    position: Vector3
    forward: Vector3
    up: Vector3
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
    """
    max_parts = 10

    body_net = multineat.NeuralNetwork()
    genotype.BuildPhenotype(body_net)

    to_explore: Queue[__Module] = Queue()
    grid = np.zeros(shape=(max_parts, max_parts, max_parts), dtype=np.uint8)

    body = BodyV1()

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
                child = __add_child(body_net, module, index, grid)
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
    :returns: (module type, rotation_index)
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
    rotation_index = rotation_probs.index(min(rotation_probs))

    return module_type, rotation_index


def __add_child(
    body_net: multineat.NeuralNetwork,
    module: __Module,
    attachment_index: int,
    grid: NDArray[np.uint8],
) -> __Module | None:
    attachment_point = module.module_reference.attachment_points[attachment_index]
    forward = __rotate(module.forward, module.up, attachment_point.rotation)
    position = module.position + forward
    chain_length = module.chain_length + 1

    # if grid cell is occupied, don't make a child
    # else, set cell as occupied

    if grid[tuple(position)] > 0:
        return None
    grid[tuple(position)] += 1

    child_type, child_rotation = __evaluate_cppn(body_net, position, chain_length)
    child_orientation = __make_quaternion_from_index(child_rotation)
    if child_type is None:
        return None
    up = __rotate(module.up, forward, child_orientation)

    child = child_type(child_orientation)
    module.module_reference.set_child(child, attachment_index)

    return __Module(
        position,
        forward,
        up,
        chain_length,
        child,
    )


def __rotate(a: Vector3, b: Vector3, rotation: Quaternion) -> Vector3:
    """
    Rotates vector a a given angle around b.

    :param a: Vector a.
    :param b: Vector b.
    :param rotation: The rotation Quaternion.
    :returns: A copy of a, rotated.
    """
    cosangle = np.cos(rotation.angle)
    sinangle = np.sin(rotation.angle)
    x, y, z = (a * cosangle + b.cross(a) * sinangle) + b * (b.dot(a) * (1 - cosangle))
    return Vector3(np.array([x, y, z], dtype=np.int64))


def __make_quaternion_from_index(index: int) -> Quaternion:
    """
    Make a quaternion from a rotation index.

    :param index: The index for rotation.
    :return: The Quaternion.
    """
    return Quaternion.from_x_rotation((np.pi / 2) * index)
