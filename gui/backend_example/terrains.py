"""Standard terrains."""

import math

import numpy as np
import numpy.typing as npt
from noise import pnoise2
from pyrr import Vector3

from revolve2.modular_robot_simulation import Terrain
from revolve2.simulation.scene import Pose
from revolve2.simulation.scene.geometry import GeometryHeightmap, GeometryPlane
from revolve2.simulation.scene.vector2 import Vector2


def flat(size=[20.0, 20.0]) -> Terrain:
    """
    Create a flat plane terrain.

    :param size: Size of the plane.
    :returns: The created terrain.
    """
    size = Vector2(size)
    return Terrain(
        static_geometry=[
            GeometryPlane(
                pose=Pose(),
                mass=0.0,
                size=size,
            )
        ]
    )


def crater(
    size=[20, 20],
    ruggedness= 0.3,
    curviness=5,
    granularity_multiplier= .5,
) -> Terrain:
    r"""
    Create a crater-like terrain with rugged floor using a heightmap.

    It will look like::

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
    num_edges = 100  # arbitrary constant to get a nice number of edges

    num_edges = (
        int(num_edges * size[0] * granularity_multiplier),
        int(num_edges * size[1] * granularity_multiplier),
    )
    OCTAVE = 10
    C1 = 4.0  # arbitrary constant to get nice noise

    rugged = np.fromfunction(
        np.vectorize(
            lambda y, x: pnoise2(
                x / num_edges[0] * C1 * size[0] * granularity_multiplier,
                y / num_edges[1] * C1 * size[1] * granularity_multiplier,
                OCTAVE,
            ),
            otypes=[float],
        ),
        num_edges,
        dtype=float,
    )

    bowl = bowl_heightmap()

    max_height = ruggedness + curviness
    if max_height == 0.0:
        heightmap = np.zeros(num_edges)
        max_height = 1.0
    else:
        heightmap = (ruggedness * rugged + curviness * bowl) / (ruggedness + curviness)

    return Terrain(
        static_geometry=[
            GeometryHeightmap(
                pose=Pose(),
                mass=0.0,
                size=Vector3([size[0], size[1], max_height]),
                base_thickness=0.01 + ruggedness,
                heights=heightmap,
            )
        ]
    )


def rugged_heightmap(
    size: tuple[float, float] = [20, 20],
    num_edges: float = 100,
    density: float = 0.5,
    hillyness: float = 1,
) -> npt.NDArray[np.float_]:
    """
    Create a rugged terrain heightmap.

    It will look like::

        ..^.__,^._.-.

    Be aware: the maximum height of the heightmap is not actually 1.
    It is around [-1,1] but not exactly.

    :param size: Size of the heightmap.
    :param num_edges: How many edges to use for the heightmap.
    :param density: How coarse the ruggedness is.
    :returns: The created heightmap as a 2 dimensional array.
    """

    num_edges = (
        int(num_edges * size[0] * density),
        int(num_edges * size[1] * density),
    )
    OCTAVE = 10
    C1 = 4.0  # arbitrary constant to get nice noise

    heightmap = np.fromfunction(
        np.vectorize(
            lambda y, x: np.clip(
                pnoise2(
                    x / num_edges[0] * C1 * size[0] * density,
                    y / num_edges[1] * C1 * size[1] * density,
                    OCTAVE,
                ) * hillyness,
                0, 1
            ),
            otypes=[float]
        ),
        num_edges,
        dtype=float,
    )

    max_height = 1

    return Terrain(
        static_geometry=[
            GeometryHeightmap(
                pose=Pose(),
                mass=0.0,
                size=Vector3([size[0], size[1], max_height]),
                base_thickness=0.1,
                heights=heightmap,
            )
        ]
    )


def bowl_heightmap(
    size: tuple[float, float] = [20, 20],
    granularity_multiplier: float = .5,
) -> Terrain:
    r"""
    Create a terrain heightmap in the shape of a bowl.

    It will look like::

        |         |
         \       /
          '.___.'

    The height of the edges of the bowl is 1.0 and the center is 0.0.

    :param size: Size of the terrain.
    :param granularity_multiplier: Multiplier for how many edges are used in the heightmap.
    :returns: The created terrain.
    """

    NUM_EDGES = 100  # arbitrary constant to get a nice number of edges

    num_edges = (
        int(NUM_EDGES * size[0] * granularity_multiplier),
        int(NUM_EDGES * size[1] * granularity_multiplier),
    )

    heightmap = np.fromfunction(
        np.vectorize(
            lambda y, x: (
                (x / num_edges[0] * 2.0 - 1.0) ** 2
                + (y / num_edges[1] * 2.0 - 1.0) ** 2
                if math.sqrt(
                    (x / num_edges[0] * 2.0 - 1.0) ** 2
                    + (y / num_edges[1] * 2.0 - 1.0) ** 2
                )
                <= 1.0
                else 0.0
            ),
            otypes=[float],
        ),
        num_edges,
        dtype=float,
    )

    max_height = 1  # The height of the edges of the bowl

    return Terrain(
        static_geometry=[
            GeometryHeightmap(
                pose=Pose(),
                mass=0.0,
                size=Vector3([size[0], size[1], max_height]),
                base_thickness=0.01,
                heights=heightmap,
            )
        ]
    )
