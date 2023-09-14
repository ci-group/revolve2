# -*- coding: utf-8 -*-
"""Represents a 2 dimensional Vector.

The Vector2 class is based on the pyrr implementation of vetors

"""
import numpy as np
from numbers import Number
from multipledispatch import dispatch
from pyrr.objects.base import BaseObject, BaseVector, BaseMatrix33, NpProxy
from .. import vector2


class BaseVector2(BaseVector):
    pass


class Vector2(BaseVector2):
    _module = vector2
    _shape = (2,)

    #: The X value of this Vector.
    x = NpProxy(0)
    #: The Y value of this Vector.
    y = NpProxy(1)
    #: The X,Y values of this Vector as a numpy.ndarray.
    xy = NpProxy([0, 1])

    ########################
    # Creation
    @classmethod
    def from_vector3(cls, vector, dtype=None):
        """Create a Vector3 from a Vector4.

        Returns the Vector3 and the W component as a tuple.
        """
        vec, w = vector2.create_from_vector3(vector, dtype)
        return (cls(vec), w)

    def __new__(cls, value=None, w=0.0, dtype=None):
        if value is not None:
            obj = value
            if not isinstance(value, np.ndarray):
                obj = np.array(value, dtype=dtype)

            # matrix44
            if obj.shape in ((3, 3,)) or isinstance(obj, BaseMatrix33):
                obj = vector2.create_from_matrix33_translation(obj, dtype=dtype)
        else:
            obj = np.zeros(cls._shape, dtype=dtype)
        obj = obj.view(cls)
        return super(Vector2, cls).__new__(cls, obj)

    ########################
    # Basic Operators
    @dispatch(BaseObject)
    def __add__(self, other):
        self._unsupported_type('add', other)

    @dispatch(BaseObject)
    def __sub__(self, other):
        self._unsupported_type('subtract', other)

    @dispatch(BaseObject)
    def __mul__(self, other):
        self._unsupported_type('multiply', other)

    @dispatch(BaseObject)
    def __truediv__(self, other):
        self._unsupported_type('divide', other)

    @dispatch(BaseObject)
    def __div__(self, other):
        self._unsupported_type('divide', other)

    @dispatch((BaseObject, Number, np.number))
    def __xor__(self, other):
        self._unsupported_type('XOR', other)

    @dispatch((BaseObject, Number, np.number))
    def __or__(self, other):
        self._unsupported_type('OR', other)

    @dispatch((BaseObject, Number, np.number))
    def __ne__(self, other):
        self._unsupported_type('NE', other)

    @dispatch((BaseObject, Number, np.number))
    def __eq__(self, other):
        self._unsupported_type('EQ', other)

    ########################
    # Vectors
    @dispatch((BaseVector2, np.ndarray, list))
    def __add__(self, other):
        return Vector2(super(Vector2, self).__add__(other))

    @dispatch((BaseVector2, np.ndarray, list))
    def __sub__(self, other):
        return Vector2(super(Vector2, self).__sub__(other))

    @dispatch((BaseVector2, np.ndarray, list))
    def __mul__(self, other):
        return Vector2(super(Vector2, self).__mul__(other))

    @dispatch((BaseVector2, np.ndarray, list))
    def __truediv__(self, other):
        return Vector2(super(Vector2, self).__truediv__(other))

    @dispatch((BaseVector2, np.ndarray, list))
    def __div__(self, other):
        return Vector2(super(Vector2, self).__div__(other))

    @dispatch((BaseVector2, np.ndarray, list))
    def __xor__(self, other):
        return self.cross(other)

    @dispatch((BaseVector2, np.ndarray, list))
    def __or__(self, other):
        return self.dot(other)

    @dispatch((BaseVector2, np.ndarray, list))
    def __ne__(self, other):
        return bool(np.any(super(Vector2, self).__ne__(other)))

    @dispatch((BaseVector2, np.ndarray, list))
    def __eq__(self, other):
        return bool(np.all(super(Vector2, self).__eq__(other)))

    ########################
    # Number
    @dispatch((Number, np.number))
    def __add__(self, other):
        return Vector2(super(Vector2, self).__add__(other))

    @dispatch((Number, np.number))
    def __sub__(self, other):
        return Vector2(super(Vector2, self).__sub__(other))

    @dispatch((Number, np.number))
    def __mul__(self, other):
        return Vector2(super(Vector2, self).__mul__(other))

    @dispatch((Number, np.number))
    def __truediv__(self, other):
        return Vector3(super(Vector2, self).__truediv__(other))

    @dispatch((Number, np.number))
    def __div__(self, other):
        return Vector2(super(Vector2, self).__div__(other))

    ########################
    # Methods and Properties
    @property
    def inverse(self):
        """Returns the opposite of this vector.
        """
        return Vector2(-self)

    @property
    def vector2(self):
        return self
