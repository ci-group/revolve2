from dataclasses import dataclass
from queue import Queue
from typing import Any

import matplotlib.pyplot as plt
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
    visualize: bool = False,  # Add a flag to control visualization
) -> BodyV2:
    """
    Develop a CPPNWIN genotype into a modular robot body with step-by-step visualization.

    :param genotype: The genotype to create the body from.
    :param visualize: Whether to visualize the body development process.
    :returns: The created body.
    """
    max_parts = 20  # Determine the maximum parts available for a robot's body.
    body_net = (
        multineat.NeuralNetwork()
    )  # Instantiate the CPPN network for body construction.
    genotype.BuildPhenotype(body_net)  # Build the CPPN from the genotype of the robot.
    to_explore: Queue[__Module] = Queue()  # Queue used to build the robot.
    grid = np.zeros(
        shape=(max_parts * 2 + 1, max_parts * 2 + 1, max_parts * 2 + 1), dtype=np.uint8
    )
    body = BodyV2()

    core_position = Vector3(
        [max_parts + 1, max_parts + 1, max_parts + 1], dtype=np.int_
    )
    grid[tuple(core_position)] = 1
    part_count = 1

    for attachment_face in body.core_v2.attachment_faces.values():
        to_explore.put(
            __Module(
                core_position,
                Vector3([0, -1, 0]),
                Vector3([0, 0, 1]),
                0,
                attachment_face,
            )
        )

    # Prepare for visualization if enabled
    if visualize:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        plt.show(block=False)

    while not to_explore.empty():
        module = to_explore.get()

        for attachment_point_tuple in module.module_reference.attachment_points.items():
            if part_count < max_parts:
                child = __add_child(body_net, module, attachment_point_tuple, grid)
                if child is not None:
                    to_explore.put(child)
                    part_count += 1
                    if visualize:
                        __visualize_structure(grid, ax)

    if visualize:
        plt.pause(0.001)  # Allow the plot to update smoothly

    return body


def __evaluate_cppn(
    body_net: multineat.NeuralNetwork,
    position: Vector3[np.int_],
    chain_length: int,
) -> tuple[Any, float]:
    """
    Get module type and orientation from a multineat CPPN network.

    :param body_net: The CPPN network.
    :param position: Position of the module.
    :param chain_length: Tree distance of the module from the core.
    :returns: (module type, angle)
    """
    x, y, z = position
    assert isinstance(
        x, np.int_
    ), f"Error: The position is not of type int. Type: {type(x)}."
    body_net.Input([1.0, x, y, z, chain_length])  # 1.0 is the bias input
    body_net.ActivateAllLayers()
    outputs = body_net.Output()

    """We select the module type for the current position using the first output of the CPPN network."""
    types = [None, BrickV2, ActiveHingeV2]
    target_idx = max(0, int(outputs[0] * len(types) - 1e-6))
    module_type = types[target_idx]

    """
    Here we get the rotation of the module from the second output of the CPPN network.
    
    The output ranges between [0,1] and we have 4 rotations available (0, 90, 180, 270).
    """
    angle = max(0, int(outputs[0] * 4 - 1e-6)) * (np.pi / 2.0)

    return module_type, angle


def __add_child(
    body_net: multineat.NeuralNetwork,
    module: __Module,
    attachment_point_tuple: tuple[int, AttachmentPoint],
    grid: NDArray[np.uint8],
) -> __Module | None:
    attachment_index, attachment_point = attachment_point_tuple

    """Here we adjust the forward facing direction, and the position for the new potential module."""
    forward = __rotate(module.forward, module.up, attachment_point.orientation)
    position = __vec3_int(module.position + forward)
    chain_length = module.chain_length + 1

    """If grid cell is occupied, we don't make a child."""
    if grid[tuple(position)] > 0:
        return None

    """Now we adjust the position for the potential new module to fit the attachment point of the parent."""
    new_pos = np.array(np.round(position + attachment_point.offset), dtype=np.int64)
    child_type, angle = __evaluate_cppn(body_net, new_pos, chain_length)

    """Check if we can place a child and if so, create the module."""
    can_set = module.module_reference.can_set_child(attachment_index)
    if (child_type is None) or (not can_set):
        return None

    """Now we know we want a child on the parent and we instantiate it, add the position to the grid and adjust the up direction for the new module."""
    child = child_type(angle)
    grid[tuple(position)] += 1
    up = __rotate(module.up, forward, Quaternion.from_eulers([angle, 0, 0]))
    module.module_reference.set_child(child, attachment_index)

    return __Module(position, forward, up, chain_length, child)

  
def __rotate(a: Vector3, b: Vector3, rotation: Quaternion) -> Vector3:
    """
    Rotate vector a, a given angle around b.
    
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
    return Vector3(list(map(lambda v: int(round(v)), vector)), dtype=np.int64)

  
def __visualize_structure(grid: NDArray[np.uint8], ax: plt.Axes) -> None:
    """
    Visualize the structure of the robot's body using Matplotlib.

    :param grid: The 3D grid containing the robot body.
    :param ax: The Matplotlib Axes3D object to draw on.
    """
    ax.clear()
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    # Get the occupied grid positions
    x, y, z = np.nonzero(grid)

    ax.scatter(x, y, z, c="r", marker="o")
    plt.draw()
    plt.pause(0.5)