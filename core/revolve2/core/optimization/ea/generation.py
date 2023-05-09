from typing import (
    TYPE_CHECKING,
    Any,
    ClassVar,
    Dict,
    ForwardRef,
    Generic,
    Type,
    TypeVar,
)

import sqlalchemy
import sqlalchemy.orm as orm
from revolve2.core._util.init_subclass_get_generic_args import (
    init_subclass_get_generic_args,
)
from revolve2.core.database import HasId
from typing_extensions import Self

TPopulation = TypeVar("TPopulation")


class Generation(HasId, orm.MappedAsDataclass, Generic[TPopulation]):
    # -------------------------------------
    # Class members interesting to the user
    # -------------------------------------
    if TYPE_CHECKING:
        generation_index: orm.Mapped[int] = orm.mapped_column(
            nullable=False, unique=True
        )
        population_id: orm.Mapped[int] = orm.mapped_column(
            sqlalchemy.ForeignKey("population.id"), nullable=False, init=False
        )
        population: orm.Mapped[TPopulation] = orm.relationship()

    # ----------------------
    # Implementation details
    # ----------------------
    else:

        @orm.declared_attr
        def generation_index(cls) -> orm.Mapped[int]:
            return cls.__generation_index_impl()

        @orm.declared_attr
        def population_id(cls) -> orm.Mapped[int]:
            return cls.__population_id_impl()

        @orm.declared_attr
        def population(cls) -> orm.Mapped[TPopulation]:
            return cls.__population_impl()

    __type_tpopulation: ClassVar[Type[TPopulation]]  # type: ignore[misc]

    def __init_subclass__(cls: Type[Self], /, **kwargs: Dict[str, Any]) -> None:
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
