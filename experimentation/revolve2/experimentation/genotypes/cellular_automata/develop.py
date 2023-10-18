import numpy as np
import math
from queue import Queue
from typing import Any
from dataclasses import dataclass

from revolve2.modular_robot import ActiveHinge, Body, Brick, Core, Module, MorphologicalMeasures
from _directions import Directions
from ca_genotype import CAGenotype



@dataclass
class __Module:
    position: tuple[int, int, int]
    forward: tuple[int, int, int]
    up: tuple[int, int, int]
    chain_length: int
    module_reference: Module


def develop(
    genotype: CAGenotype,
) -> Body:
    """


    :param genotype: 
    :returns:
    :raises RuntimeError: In case a module is encountered that is not supported.
    """
    max_parts = 10

    to_explore: Queue[__Module] = Queue()
    grid: set[tuple[int, int]] = set()

    body = Body()
    ca_grid = genotype.ca_grid
    core_pos = genotype.core_position

    to_explore.put(__Module((core_pos[0], core_pos[1], 0), (0, -1, 0), (0, 0, 1), 0, body.core))
    grid.add((0, 0, 0))
    part_count = 1

    while not to_explore.empty():
        module = to_explore.get()
        print(module)

        children: list[tuple[int, int]] = []  # child index, rotation

        if isinstance(module.module_reference, Core):
            children.append((Directions.FRONT, 0))
            children.append((Directions.LEFT, 1))
            children.append((Directions.BACK, 2))
            children.append((Directions.RIGHT, 3))
        elif isinstance(module.module_reference, Brick):
            children.append((Directions.FRONT, 0))
            children.append((Directions.LEFT, 1))
            children.append((Directions.RIGHT, 3))
        elif isinstance(module.module_reference, ActiveHinge):
            children.append((Directions.FRONT, 0))
        else:  # Should actually never arrive here but just checking module type to be sure
            raise RuntimeError()

        for index, rotation in children:
            if part_count < max_parts:
                child = __add_child(ca_grid, module, index, rotation, grid)
                if child is not None:
                    to_explore.put(child)
                    part_count += 1

    body.finalize()
    return body


def __module_from_ca(
    position: tuple[int, int, int],
    chain_length: int,
    ca_grid,
) -> tuple[Any, int]:
    """

    :param position: Position of the module.
    :param chain_length: Tree distance of the module from the core.
    :param ca_grid: 
    :returns: (module type, orientation)
    """

    rotation = 0
    
    print('current position on ca_grid: ', position)
    print('ca_grid value at position: ', ca_grid[position[0]][position[1]])

    if ca_grid[position[0]][position[1]] == 1:
        module_type = Brick
    elif ca_grid[position[0]][position[1]] == 2:
        module_type = ActiveHinge
    else:
        module_type = None

    return (module_type, rotation)


def __add_child(
    ca_grid,
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

    child_type, orientation = __module_from_ca(position, chain_length, ca_grid)
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

