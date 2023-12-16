from dataclasses import dataclass
from queue import Queue
from typing import Any

import multineat
import numpy as np
from numpy.typing import NDArray
from pyrr import Quaternion, Vector3

from revolve2.modular_robot.body import AttachmentPoint, Module
from revolve2.modular_robot.body.v2 import ActiveHingeV2, BodyV2, BrickV2


@dataclass
class __Module:
    position: Vector3[np.int_]
    forward: Vector3[np.int_]
    up: Vector3[np.int_]
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

    body = BodyV2()

    v2_core = body.core_v2

    for attachment_face in v2_core.attachment_faces.values():
        to_explore.put(
            __Module(
                Vector3([0, 0, 0]),
                Vector3([0, -1, 0]),
                Vector3([0, 0, 1]),
                0,
                attachment_face,
            )
        )
    grid[0, 0, 0] = 1
    part_count = 1

    while not to_explore.empty():
        module = to_explore.get()

        for attachment_point_tuple in module.module_reference.attachment_points.items():
            if part_count < max_parts:
                child = __add_child(body_net, module, attachment_point_tuple, grid)
                if child is not None:
                    to_explore.put(child)
                    part_count += 1
    return body


def __evaluate_cppn(
    body_net: multineat.NeuralNetwork,
    position: Vector3[np.int_],
    chain_length: int,
) -> tuple[Any, int]:
    """
    Get module type and orientation from a multineat CPPN network.

    :param body_net: The CPPN network.
    :param position: Position of the module.
    :param chain_length: Tree distance of the module from the core.
    :returns: (module type, orientation)
    """
    x, y, z = position
    assert isinstance(
        x, np.int_
    ), f"Error: The position is not of type int. Type: {type(x)}."
    body_net.Input([1.0, x, y, z, chain_length])  # 1.0 is the bias input
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
    attachment_point_tuple: tuple[int, AttachmentPoint],
    grid: NDArray[np.uint8],
) -> __Module | None:
    attachment_index, attachment_point = attachment_point_tuple

    forward = __rotate(module.forward, module.up, attachment_point.orientation)
    position = module.position + forward
    chain_length = module.chain_length + 1

    # if grid cell is occupied, don't make a child
    # else, set cell as occupied
    grid_pos = np.round(position)
    if grid[tuple(grid_pos)] > 0:
        return None
    grid[grid_pos] += 1

    new_pos = np.array(np.round(position + attachment_point.offset), dtype=np.int64)
    child_type, child_rotation = __evaluate_cppn(body_net, new_pos, chain_length)
    child_orientation = __make_quaternion_from_index(child_rotation)
    if child_type is None or not module.module_reference.can_set_child(
        child := child_type(child_orientation.angle), attachment_index
    ):
        return None

    up = __rotate(module.up, forward, child_orientation)
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
    :param rotation: The rotation quaternion.
    :returns: A copy of a, rotated.
    """
    cosangle = np.cos(rotation.angle)
    sinangle = np.sin(rotation.angle)
    x, y, z = (a * cosangle + b.cross(a) * sinangle) + b * (b.dot(a) * (1 - cosangle))
    return Vector3(np.array([x, y, z], dtype=np.int64))


def __make_quaternion_from_index(index: int) -> Quaternion:
    """
    Make a quaternion from a rotation index.

    :param index: The index of rotation.
    :return: The Quaternion.
    """
    return Quaternion.from_x_rotation((np.pi / 2) * index)
