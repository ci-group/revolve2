from dataclasses import dataclass
from queue import Queue
from typing import Any

import multineat
import numpy as np
from numpy.typing import NDArray
from pyrr import Quaternion, Vector3
import pickle
import time

from revolve2.modular_robot.body import AttachmentPoint, Module
from revolve2.modular_robot.body.v2 import (
    ActiveHingeV2,
    AttachmentFaceCoreV2,
    BodyV2,
    BrickV2,
)


class _Voxel:
    _size: int
    _aabb: tuple[Vector3[np.int_], Vector3[np.int_]]
    _attachment_point: dict[float, Vector3[np.int_]]

    def __init__(self, size: int) -> None:
        """
        Initialize the Voxel.

        :param size: The voxels size.
        """
        self._size = size
        self._aabb = (Vector3([0, 0, 0]), Vector3([size, size, size]))
        self._attachment_point = {i: Vector3([size if i > 1 else 0, 0 if i % 3 == 0 else size, 0]) for i in range(4)}

    def __add__(self, other: 'Vector3') -> None:
        self._aabb = tuple([elem + other for elem in self._aabb])

    def rotate(self, axis: Vector3, angle: Quaternion) -> None:
        self._aabb = tuple([_rotate(elem, axis, angle) for elem in self._aabb])
        self._attachment_point = {i: _rotate(vec, axis, angle) for i, vec in self._attachment_point.items()}

    def get_attachment_point(self, angle: float) -> Vector3:
        return self._attachment_point[np.rint(angle/np.pi*2)]

    @property
    def aabb(self) -> tuple[Vector3[np.int_], Vector3[np.int_]]:
        return self._aabb

    @property
    def size(self) -> int:
        return self._size


@dataclass
class __Module:
    position: Vector3[np.int_]
    forward: Vector3[np.int_]
    up: Vector3[np.int_]
    chain_length: int
    module_reference: Module
    parent_voxel: _Voxel


def develop(
        genotype: multineat.Genome,
) -> BodyV2:
    """
    Develop a CPPNWIN genotype into a modular robot body.

    It is important that the genotype was created using a compatible function.

    :param genotype: The genotype to create the body from.
    :returns: The create body.
    """
    max_parts = 40
    """
    TODO: Robot forward is [0, -1, 0]
     -> Make Voxels initialize with the same forward direction
     -> Make Voxels rotate accoringly and update AttachmentPoints
     -> Stack Voxels in correct order
    """

    body_net = multineat.NeuralNetwork()
    genotype.BuildPhenotype(body_net)

    to_explore: Queue[__Module] = Queue()

    core_voxel = _Voxel(8)
    grid = np.zeros(
        shape=(
            (max_parts * 2 + 1) * core_voxel.size,
            (max_parts * 2 + 1) * core_voxel.size,
            (max_parts * 2 + 1) * core_voxel.size,
        ),
        dtype=np.uint8,
    )

    body = BodyV2()

    v2_core = body.core_v2
    core_position = Vector3(
        [max_parts * core_voxel.size, max_parts * core_voxel.size, max_parts * core_voxel.size],
        dtype=np.int_
    )
    core_voxel + core_position
    (xo, yo, zo), (xt, yt, zt) = core_voxel.aabb

    grid[xo: xt+1, yo:yt+1, zo:zt+1] = 1
    part_count = 1
    for attachment_face in v2_core.attachment_faces.values():
        to_explore.put(
            __Module(
                position=core_position,
                forward=Vector3([0, -1, 0]),
                up=Vector3([0, 0, 1]),
                chain_length=0,
                module_reference=attachment_face,
                parent_voxel=core_voxel,
            )
        )

    while not to_explore.empty() and part_count < max_parts:
        module = to_explore.get()

        for attachment_point_tuple in module.module_reference.attachment_points.items():
            child = __add_child(body_net, module, attachment_point_tuple, grid)
            if child is not None:
                to_explore.put(child)
                part_count += 1

    with open(f"{time.time()}.pickle", "wb") as file:
        pickle.dump(grid, file)
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
    """
    Add a potential child to the modular robot.

    :param body_net: The CPPN network for evaluation of position.
    :param module: The previous module and parameters.
    :param attachment_point_tuple: The attachment point data for the child.
    :param grid: The grid for collision checking.
    :return: A new module or nothing if now child is added.
    """
    attachment_index, attachment_point = attachment_point_tuple
    forward = _rotate(module.forward, module.up, attachment_point.orientation)
    parent_offset = module.parent_voxel.get_attachment_point(attachment_point.orientation.angle)
    match module.module_reference.parent:
        case AttachmentFaceCoreV2():
            """If we are on an AttachmentFace we employ special offsets to the grid positions."""
            offset = _rotate(Vector3([(attachment_index % 3)*2, 0, 4-(attachment_index // 3) * 2]), module.up, attachment_point.orientation)
            position = __vec3_int(module.position + parent_offset + offset)
        case _:
            position = __vec3_int(module.position + parent_offset)
    voxel = _Voxel(4)
    voxel.rotate(module.up, attachment_point.orientation)
    voxel + position
    chain_length = module.chain_length + 1

    # if grid cell is occupied, don't make a child
    (xo, yo, zo), (xt, yt, zt) = voxel.aabb
    if np.sum(grid[xo: xt+1, yo:yt+1, zo:zt+1]) > 0:
        return None
    child_type, child_rotation = __evaluate_cppn(body_net, position, chain_length)

    """We check whether the CPPN wants to place a child or if the module can accept a child at the corresponding attachment_index."""
    if child_type is None or not module.module_reference.is_free(attachment_index):
        return None
    angle = child_rotation * (np.pi / 2.0)
    child = child_type(angle)
    grid[xo: xt+1, yo:yt+1, zo:zt+1] += 1

    up = _rotate(module.up, forward, Quaternion.from_eulers([angle, 0, 0]))
    module.module_reference.set_child(child, attachment_index)
    return __Module(position, forward, up, chain_length, child, voxel)


def _rotate(a: Vector3, b: Vector3, rotation: Quaternion) -> Vector3:
    """
    Rotates vector a a given angle around b.

    :param a: Vector a.
    :param b: Vector b.
    :param rotation: The quaternion for rotation.
    :returns: A copy of a, rotated.
    """
    cos_angle: int = int(round(np.cos(rotation.angle)))
    sin_angle: int = int(round(np.sin(rotation.angle)))

    vec: Vector3 = (
            a * cos_angle + sin_angle * b.cross(a) + (1 - cos_angle) * b.dot(a) * b
    )
    return vec


def __vec3_int(vector: Vector3) -> Vector3[np.int_]:
    """
    Cast a Vector3 object to an integer only Vector3.

    :param vector: The vector.
    :return: The integer vector.
    """
    x, y, z = map(lambda v: int(round(v)), vector)
    return Vector3([x, y, z], dtype=np.int64)
