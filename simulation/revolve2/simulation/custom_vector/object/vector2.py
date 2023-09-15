"""A representation of a 2d Vector."""
from __future__ import annotations

from numbers import Number
from typing import Any

import numpy as np
from pyrr.objects.base import BaseMatrix33, BaseVector, NpProxy

from .. import vector2


class BaseVector2(BaseVector):  # type:ignore
    """Base Class for Vector2."""

    pass


class Vector2(BaseVector2):
    """Represents a 2-dimensional Vector. The Vector2 class is based on the pyrr implementation of vectors."""

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
    def from_vector3(cls, vector: Any, dtype: Any = None) -> tuple[Vector2, float]:
        """
        Create a Vector2 from a Vector3.

        :param vector: The Vector3.
        :param dtype: The data-type.
        :return: The vector2.
        """
        vec, w = vector2.create_from_vector3(vector, dtype)
        return (cls(vec), w)

    def __new__(cls, value: Any = None, w: float = 0.0, dtype: Any = None) -> Any:
        """
        Make a new Vector2.

        :param value: The value of the Vector2.
        :param w: unused rest.
        :param dtype: The data-type.
        :return: A new Vector2.
        """
        if value is not None:
            obj = value
            if not isinstance(value, np.ndarray):
                obj = np.array(value, dtype=dtype)

            # matrix33
            if obj.shape in (
                (
                    3,
                    3,
                )
            ) or isinstance(obj, BaseMatrix33):
                obj = vector2.create_from_matrix33_translation(obj, dtype=dtype)
        else:
            obj = np.zeros(cls._shape, dtype=dtype)
        obj = obj.view(cls)
        return super(Vector2, cls).__new__(cls, obj)

    ########################
    # Operators
    __NMB = [Number, np.number]
    __VCT = [BaseVector2, np.ndarray, list]

    def __add__(self, other: Any) -> Vector2:  # type:ignore
        """
        Add to the existing Vector2.

        :param other: The other Vector2.
        :return: The added Vector2.
        """
        if type(other) in self.__NMB:
            return Vector2(super(Vector2, self).__add__(other))
        elif type(other) in self.__VCT:
            return Vector2(super(Vector2, self).__add__(other))
        else:
            self._unsupported_type("add", other)

    def __sub__(self, other: Any) -> Vector2:  # type:ignore
        """
        Subtract from the existing Vector2.

        :param other: The other Vector2.
        :return: The subtracted Vector2.
        """
        if type(other) in self.__NMB:
            return Vector2(super(Vector2, self).__sub__(other))
        elif type(other) in self.__VCT:
            return Vector2(super(Vector2, self).__sub__(other))
        else:
            self._unsupported_type("subtract", other)

    def __mul__(self, other: Any) -> Vector2:  # type:ignore
        """
        Multiply the existing Vector2.

        :param other: The other Vector2.
        :return: the multiplied Vector2.
        """
        if type(other) in self.__NMB:
            return Vector2(super(Vector2, self).__mul__(other))
        elif type(other) in self.__VCT:
            return Vector2(super(Vector2, self).__mul__(other))
        else:
            self._unsupported_type("multiply", other)

    def __truediv__(self, other: Any) -> Vector2:  # type:ignore
        """
        Apply the true division of the existing Vector2.

        :param other: The other Vector2.
        :return: The resulting Vector2.
        """
        if type(other) in self.__NMB:
            return Vector2(super(Vector2, self).__truediv__(other))
        elif type(other) in self.__VCT:
            return Vector2(super(Vector2, self).__truediv__(other))
        else:
            self._unsupported_type("divide", other)

    def __div__(self, other: Any) -> Vector2:  # type:ignore
        """
        Divide the existing Vector2.

        :param other: The other Vector2.
        :return: the resulting Vector2.
        """
        if type(other) in self.__NMB:
            return Vector2(super(Vector2, self).__div__(other))
        elif type(other) in self.__VCT:
            return Vector2(super(Vector2, self).__div__(other))
        else:
            self._unsupported_type("divide", other)

    def __xor__(self, other: Any) -> Any:
        """
        Calculate the cross-product.

        :param other: The other Vector2.
        :return: The cross-product.
        """
        if type(other) in self.__VCT:
            return self.cross(other)
        else:
            self._unsupported_type("XOR", other)

    def __or__(self, other: Any) -> Any:
        """
        Calculate the dot-product.

        :param other: The other Vector2.
        :return: The dot-product.
        """
        if type(other) in self.__VCT:
            return self.dot(other)
        else:
            self._unsupported_type("OR", other)

    def __ne__(self, other: Any) -> bool:  # type:ignore
        """
        Not equal to the existing Vector2.

        :param other: The other Vector2.
        :return: whether they are unequal.
        """
        if type(other) in self.__VCT:
            return bool(np.any(super(Vector2, self).__ne__(other)))
        else:
            self._unsupported_type("NE", other)

    def __eq__(self, other: Any) -> bool:  # type:ignore
        """
        Equal to the existing Vector2.

        :param other: The other Vector2.
        :return: whether they are equal.
        """
        if type(other) in self.__VCT:
            return bool(np.all(super(Vector2, self).__eq__(other)))
        else:
            self._unsupported_type("EQ", other)

    ########################
    # Methods and Properties
    @property
    def inverse(self) -> Vector2:
        """:return: the inversed Vector2."""
        return Vector2(-self)

    @property
    def vector2(self) -> Vector2:
        """:return: itself."""
        return self
