from typing import TYPE_CHECKING, Any, ClassVar, ForwardRef, Generic, Type, TypeVar

import sqlalchemy.ext.orderinglist
import sqlalchemy.orm as orm
from typing_extensions import Self

from ..._util.init_subclass_get_generic_args import init_subclass_get_generic_args
from ...database import HasId

TIndividual = TypeVar("TIndividual")


class Population(HasId, orm.MappedAsDataclass, Generic[TIndividual]):
    """
    Generic SQLAlchemy model for a population.

    Inherit from this to create your own population type.

    The generic parameter `TIndividual` refers to the user-defined individual type.
    This parameter cannot be a forward reference.

    For example::

        class MyPopulation(Base, Population[MyIndividual]):
            __tablename__ = "my_population"
    """

    # -------------------------------------
    # Class members interesting to the user
    # -------------------------------------
    if TYPE_CHECKING:
        individuals: orm.Mapped[list[TIndividual]]

    # ----------------------
    # Implementation details
    # ----------------------
    else:

        @orm.declared_attr
        def individuals(cls) -> orm.Mapped[list[TIndividual]]:  # noqa
            return cls.__individuals_impl()

    __type_tindividual: ClassVar[Type[TIndividual]]  # type: ignore[misc]

    def __init_subclass__(cls: Type[Self], /, **kwargs: dict[str, Any]) -> None:
        """
        Initialize a version of this class when it is subclassed.

        Gets the actual type of `TIndividual` and stores it for later use.

        :param kwargs: Remaining arguments passed to super.
        """
        generic_types = init_subclass_get_generic_args(cls, Population)
        assert len(generic_types) == 1
        cls.__type_tindividual = generic_types[0]
        assert not isinstance(
            cls.__type_tindividual, ForwardRef
        ), "TIndividual generic argument cannot be a forward reference."

        super().__init_subclass__(**kwargs)  # type: ignore[arg-type]

    @classmethod
    def __individuals_impl(cls) -> orm.Mapped[TIndividual]:
        return orm.relationship(
            cls.__type_tindividual,
            order_by=cls.__type_tindividual.population_index,
            collection_class=sqlalchemy.ext.orderinglist.ordering_list(
                "population_index"
            ),
        )
