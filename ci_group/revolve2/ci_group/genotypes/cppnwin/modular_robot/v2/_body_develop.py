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
    grid = np.zeros(shape=(max_parts*2+1, max_parts*2+1, max_parts*2+1), dtype=np.uint8)

    body = BodyV2()

    v2_core = body.core_v2
    core_position = Vector3([max_parts+1, max_parts+1, max_parts+1], dtype=np.int_)
    grid[tuple(core_position)] = 1
    part_count = 1

    for attachment_face in v2_core.attachment_faces.values():
        to_explore.put(
            __Module(
                core_position,
                Vector3([0, -1, 0]),
                Vector3([0, 0, 1]),
                0,
                attachment_face,
            )
        )

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
    :returns: (module type, rotation_index)
    """
    x, y, z = position
    assert isinstance(
        x, np.int_
    ), f"Error: The position is not of type int. Type: {type(x)}."
    body_net.Input([1.0, x, y, z, chain_length])  # 1.0 is the bias input
    body_net.ActivateAllLayers()
    outputs = body_net.Output()

    # get module type from output probabilities
    type_probs = list(outputs[:3])
    types = [None, BrickV2, ActiveHingeV2]
    module_type = types[type_probs.index(min(type_probs))]

    # get rotation from output probabilities
    rotation_probs = list(outputs[3:5])
    rotation_index = rotation_probs.index(min(rotation_probs))

    return module_type, rotation_index


def __add_child(
    body_net: multineat.NeuralNetwork,
    module: __Module,
    attachment_point_tuple: tuple[int, AttachmentPoint],
    grid: NDArray[np.uint8],
) -> __Module | None:
    attachment_index, attachment_point = attachment_point_tuple

    forward = __rotate(module.forward, module.up, attachment_point.orientation)
    position = __vec3_int(module.position + forward)
    chain_length = module.chain_length + 1

    # if grid cell is occupied, don't make a child
    # else, set cell as occupied
    grid_pos = np.round(position)
    if grid[tuple(position)] > 0:
        return None
    grid[tuple(position)] += 1

    new_pos = np.array(np.round(position + attachment_point.offset), dtype=np.int64)
    child_type, child_rotation = __evaluate_cppn(body_net, new_pos, chain_length)
    angle = child_rotation * (np.pi / 2.0)
    child = child_type(angle)
    if child_type is None or not module.module_reference.can_set_child(child, attachment_index):
        return None

    up = __rotate(module.up, forward, Quaternion.from_eulers([angle, 0, 0]))
    module.module_reference.set_child(child, attachment_index)

    return __Module(
        position,
        forward,
        up,
        chain_length,
        child,
    )


def __rotate(
        a: Vector3, b: Vector3, rotation: Quaternion
) -> Vector3:
    """
    Rotates vector a a given angle around b.

    :param a: Vector a.
    :param b: Vector b.
    :param rotation: The quaternion for rotation.
    :returns: A copy of a, rotated.
    """
    cos_angle: int = int(round(np.cos(rotation.angle)))
    sin_angle: int = int(round(np.sin(rotation.angle)))

    vec: Vector3 = a * cos_angle + sin_angle * b.cross(a) + (1 - cos_angle) * b.dot(a) * b
    return vec


def __vec3_int(vector: Vector3) -> Vector3[np.int_]:
    """
    Cast a Vector3 object to an integer only Vector3.

    :param vector: The vector.
    :return: The integer vector.
    """
    x, y, z = map(lambda v: int(round(v)), vector)
    return Vector3([x, y, z], dtype=np.int64)

