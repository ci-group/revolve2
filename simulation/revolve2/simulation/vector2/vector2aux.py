# -*- coding: utf-8 -*-
"""Provides functions for creating and manipulating 2D vectors."""
from __future__ import absolute_import, division, print_function, unicode_literals

from typing import Any

import numpy as np

# import common vector operations
from pyrr.utils import parameters_as_numpy_arrays


def create(
    x: float = 0.0, y: float = 0.0, dtype: Any = None
) -> np.ndarray:  # type:ignore
    """
    Create a Vector2 object.

    # noqa: DAR401

    :param x: x size.
    :param y: y size.
    :param dtype: data-type.
    :return: The Vector.
    """
    if isinstance(x, (list, np.ndarray)):
        raise ValueError("Function requires non-list arguments")
    return np.array([x, y], dtype=dtype)


def create_unit_length_x(dtype: Any = None) -> np.ndarray:  # type:ignore
    """
    Create a x unit Vector2.

    :param dtype: data-type.
    :return: The Vector.
    """
    return np.array([1.0, 0.0], dtype=dtype)


def create_unit_length_y(dtype: Any = None) -> np.ndarray:  # type:ignore
    """
    Create a y unit Vector2.

    :param dtype: data-type.
    :return: The Vector.
    """
    return np.array([0.0, 1.0], dtype=dtype)


@parameters_as_numpy_arrays("mat")  # type:ignore
def create_from_matrix33_translation(
    mat: Any, dtype: Any = None
) -> np.ndarray:  # type:ignore
    """
    Create a Vector2 from a 3x3 Matrix.

    :param mat: The 3x3 Matrix.
    :param dtype: The data-type.
    :return: The Vector2.
    """
    return np.array(mat[2, :2], dtype=dtype)


class index:
    """Index class for the Vector2."""

    #: The index of the X value within the vector
    x = 0

    #: The index of the Y value within the vector
    y = 1


class unit:
    """Unit class for the Vector2."""

    #: A vector of unit length in the X-axis. (1.0, 0.0, 0.0)
    x = create_unit_length_x()

    #: A vector of unit length in the Y-axis. (0.0, 1.0, 0.0)
    y = create_unit_length_y()
