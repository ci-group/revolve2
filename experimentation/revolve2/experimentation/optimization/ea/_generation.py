from typing import TYPE_CHECKING, Any, ClassVar, ForwardRef, Generic, Type, TypeVar

import sqlalchemy
import sqlalchemy.orm as orm
from typing_extensions import Self

from ..._util.init_subclass_get_generic_args import init_subclass_get_generic_args
from ...database import HasId

TPopulation = TypeVar("TPopulation")


class Generation(HasId, orm.MappedAsDataclass, Generic[TPopulation]):
    """
    Generic SQLAlchemy model for a generation.

    Inherit from this to create your own generation type and table.

    The generic parameter `TPopulation` refers to the user-defined population type.
    This parameter cannot be a forward reference.

    For example::

        class MyGeneration(Base, Generation[MyPopulation]):
            __tablename__ = "my_generation"
    """

    # -------------------------------------
    # Class members interesting to the user
    # -------------------------------------
    if TYPE_CHECKING:
        generation_index: orm.Mapped[int] = orm.mapped_column(
            nullable=False, unique=True
        )
        _population_id: orm.Mapped[int] = orm.mapped_column(
            "population_id",
            sqlalchemy.ForeignKey("population.id"),
            nullable=False,
            init=False,
        )
        population: orm.Mapped[TPopulation] = orm.relationship()

    # ----------------------
    # Implementation details
    # ----------------------
    else:

        @orm.declared_attr
        def generation_index(cls) -> orm.Mapped[int]:  # noqa
            return cls.__generation_index_impl()

        @orm.declared_attr
        def _population_id(cls) -> orm.Mapped[int]:
            return cls.__population_id_impl()

        @orm.declared_attr
        def population(cls) -> orm.Mapped[TPopulation]:  # noqa
            return cls.__population_impl()

    __type_tpopulation: ClassVar[Type[TPopulation]]  # type: ignore[misc]

    def __init_subclass__(cls: Type[Self], /, **kwargs: dict[str, Any]) -> None:
        """
        Initialize a version of this class when it is subclassed.

        Gets the actual type of `TPopulation` and stores it for later use.
        :param kwargs: Remaining arguments passed to super.
        """
        generic_types = init_subclass_get_generic_args(cls, Generation)
        assert len(generic_types) == 1
        cls.__type_tpopulation = generic_types[0]
        assert not isinstance(
            cls.__type_tpopulation, ForwardRef
        ), "TPopulation generic argument cannot be a forward reference."

        super().__init_subclass__(**kwargs)  # type: ignore[arg-type]

    @classmethod
    def __generation_index_impl(cls) -> orm.Mapped[int]:
        return orm.mapped_column(nullable=False, unique=True)

    @classmethod
    def __population_id_impl(cls) -> orm.Mapped[int]:
        return orm.mapped_column(
            sqlalchemy.ForeignKey(f"{cls.__type_tpopulation.__tablename__}.id"),
            nullable=False,
            init=False,
        )

    @classmethod
    def __population_impl(cls) -> orm.Mapped[TPopulation]:
        return orm.relationship(cls.__type_tpopulation)
