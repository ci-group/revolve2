# -*- coding: utf-8 -*-
"""Provides functions for creating and manipulating 3D vectors.
"""
from __future__ import absolute_import, division, print_function, unicode_literals
import numpy as np

# import common vector operations
from pyrr.vector import *


def create(x=0., y=0., dtype=None):
    if isinstance(x, (list, np.ndarray)):
        raise ValueError('Function requires non-list arguments')
    return np.array([x,y], dtype=dtype)

def create_unit_length_x(dtype=None):
    return np.array([1.0, 0.0], dtype=dtype)

def create_unit_length_y(dtype=None):
    return np.array([0.0, 1.0], dtype=dtype)


@parameters_as_numpy_arrays('vector')
def create_from_vector3(vector, dtype=None):
    """Returns a vector3 and the W component as a tuple.
    """
    dtype = dtype or vector.dtype
    return (np.array([vector[0], vector[1]], dtype=dtype), vector[2])

@parameters_as_numpy_arrays('mat')
def create_from_matrix33_translation(mat, dtype=None):
    return np.array(mat[2, :2], dtype=dtype)


class index:
    #: The index of the X value within the vector
    x = 0

    #: The index of the Y value within the vector
    y = 1



class unit:
    #: A vector of unit length in the X-axis. (1.0, 0.0, 0.0)
    x = create_unit_length_x()

    #: A vector of unit length in the Y-axis. (0.0, 1.0, 0.0)
    y = create_unit_length_y()
