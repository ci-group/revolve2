from __future__ import annotations

from dataclasses import dataclass
from typing import Type


@dataclass
class DbId:
    """
    Identifier in a database.

    Can can create children identifiers, forming a tree structure.
    """

    __fullname: str

    @staticmethod
    def __is_lower_alpha_or_raise(name: str) -> None:
        if not (name.isalnum() and name.islower()):
            raise ValueError("DbId name can only consist of lower case letters.")

    @classmethod
    def root(cls: Type[DbId], name: str) -> DbId:
        """
        Create the root of an id tree.

        :param name: Name of the root id.
        :returns: The created id.
        """
        cls.__is_lower_alpha_or_raise(name)
        return cls(name)

    @property
    def fullname(self: DbId) -> str:
        """
        Get the string representation of this id.

        :returns: The representation.
        """
        return self.__fullname

    def branch(self: DbId, name: str) -> DbId:
        """
        Create a child id.

        :param name: Identifier for the child. Make sure this is unique relative to the current id.
        :returns: The created id.
        """
        self.__is_lower_alpha_or_raise(name)
        return DbId(f"{self.__fullname}_{name}")
