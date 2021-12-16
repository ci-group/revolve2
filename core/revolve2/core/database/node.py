from abc import ABC, abstractmethod

from .list import List as DbList
from .object import Object
from .transaction import Transaction


class Node(ABC):
    """
    Represents a node in a database.
    Can be either an object, a list, or unitialized.
    """

    @abstractmethod
    @property
    def is_stub(self) -> bool:
        """
        If node is not yet linked to the database but a stub created by the user.
        """
        pass

    @abstractmethod
    def make_object(self, tnx: Transaction, object: Object) -> None:
        """
        Make this node an object. Raises DatabaseError if node is not uninitialized.
        """
        pass

    @abstractmethod
    def make_list(self, tnx: Transaction) -> DbList:
        """
        Make this node a list. Raises DatabaseError if node is not uninitialized.
        """
        pass

    @abstractmethod
    def is_uninitialized(self, tnx: Transaction) -> bool:
        """
        Checks if this node is uninitialized.
        """

    @abstractmethod
    def as_object(self, tnx: Transaction) -> Object:
        """
        Read this node as an object, or raise an error if this node is not an object.
        """
        pass

    @abstractmethod
    def as_list(self, tnx: Transaction) -> DbList:
        """
        Read this node as a list, or raise an error if this node is not a list.
        """
        pass
