"""Interface and implementation of geometries."""

from dataclasses import dataclass

import numpy as np
import numpy.typing as npt
from pyrr import Quaternion, Vector3


@dataclass
class Geometry:
    """Base for all geometries."""

    position: Vector3
    orientation: Quaternion


@dataclass
class Plane(Geometry):
    """A flat plane geometry."""

    size: Vector3  # z is ignored because a plane has 0 thickness # TODO use some kind of vector3
    color: Vector3 = Vector3([0.2, 0.2, 0.2])


@dataclass
class Heightmap(Geometry):
    """
    A heightmap geometry.

    Similarly to the `Plane` geometry, x and y of `size` define the space the heighmap encompasses.
    The z-coordinate defines the height of a heightmap edge when it's value is maximum.
    `heights` defines the edge of the heighmap. Values much lie between 0.0 and 1.0, inclusive.
    `base_thickness` defines the thickness of the box below the heighmap, which is requires for proper collision detection in some runners.
    """

    size: Vector3
    base_thickness: float
    heights: npt.NDArray[np.float_]  # MxN matrix. outer list is x, inner list is y
    color: Vector3 = Vector3([0.2, 0.2, 0.2])
