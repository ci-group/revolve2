"""Standard terrains."""

import math
from typing import Tuple

import numpy as np
import numpy.typing as npt
from noise import pnoise2
from pyrr import Quaternion, Vector3
from revolve2.core.physics import Terrain
from revolve2.core.physics.running import geometry


def flat(size: Vector3 = Vector3([20.0, 20.0, 0.0])) -> Terrain:
    """
    Create a flat plane terrain.

    :param size: Size of the plane.
    :returns: The created terrain.
    """
    return Terrain(
        static_geometry=[
            geometry.Plane(
                position=Vector3(),
                orientation=Quaternion(),
                size=size,
            )
        ]
    )


def crater(
    size: Tuple[float, float],
    ruggedness: float,
    curviness: float,
    granularity_multiplier: float = 1.0,
) -> Terrain:
    r"""
    Create a crater-like terrain with rugged floor using a heightmap.

    |            |
     \_        .'
       '.,^_..'

    A combination of the rugged and bowl heightmaps.

    :param size: Size of the crater.
    :param ruggedness: How coarse the ground is.
    :param curviness: Height of the edges of the crater.
    :param granularity_multiplier: Multiplier for how many edges are used in the heightmap.
    :returns: The created terrain.
    """
    NUM_EDGES = 100  # arbitrary constant to get a nice number of edges

    num_edges = (
        int(NUM_EDGES * size[0] * granularity_multiplier),
        int(NUM_EDGES * size[1] * granularity_multiplier),
    )

    rugged = rugged_heightmap(
        size=size,
        num_edges=num_edges,
        density=1.5,
    )
    bowl = bowl_heighmap(num_edges=num_edges)

    max_height = ruggedness + curviness
    if max_height == 0.0:
        heightmap = np.zeros(num_edges)
        max_height = 1.0
    else:
        heightmap = (ruggedness * rugged + curviness * bowl) / (ruggedness + curviness)

    return Terrain(
        static_geometry=[
            geometry.Heightmap(
                position=Vector3(),
                orientation=Quaternion(),
                size=Vector3([size[0], size[1], max_height]),
                base_thickness=0.1 + ruggedness,
                heights=heightmap,
            )
        ]
    )


def rugged_heightmap(
    size: Tuple[float, float],
    num_edges: Tuple[int, int],
    density: float = 1.0,
) -> npt.NDArray[np.float_]:
    """
    Create a rugged terrain heightmap.

    ..^.__,^._.-.

    TODO The maximum height of the heightmap is not actually 1, but it maybe should be.
    It is around [-1,1] but not exactly.

    :param size: Size of the heightmap.
    :param num_edges: How many edges to use for the heightmap.
    :param density: How coarse the ruggedness is.
    :returns: The created heightmap as a 2 dimensional array.
    """
    OCTAVE = 10
    C1 = 4.0  # arbitrary constant to get nice noise

    return np.fromfunction(
        np.vectorize(
            lambda y, x: pnoise2(
                x / num_edges[0] * C1 * size[0] * density,
                y / num_edges[1] * C1 * size[1] * density,
                OCTAVE,
            ),
            otypes=[float],
        ),
        num_edges,
        dtype=float,
    )


def bowl_heighmap(
    num_edges: Tuple[int, int],
) -> npt.NDArray[np.float_]:
    r"""
    Create a terrain heightmap in the shape of a bowl.

    |         |
     \       /
      '.___.'

    The height of the edges of the bowl is 1.0 and the center is 0.0.

    :param num_edges: How many edges to use for the heightmap.
    :returns: The created heightmap as a 2 dimensional array.
    """
    return np.fromfunction(
        np.vectorize(
            lambda y, x: (x / num_edges[0] * 2.0 - 1.0) ** 2
            + (y / num_edges[1] * 2.0 - 1.0) ** 2
            if math.sqrt(
                (x / num_edges[0] * 2.0 - 1.0) ** 2
                + (y / num_edges[1] * 2.0 - 1.0) ** 2
            )
            <= 1.0
            else 0.0,
            otypes=[float],
        ),
        num_edges,
        dtype=float,
    )
