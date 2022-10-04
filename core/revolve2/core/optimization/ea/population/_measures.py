from __future__ import annotations

from dataclasses import dataclass
from typing import (
    Any,
    Callable,
    Protocol,
    Type,
    TypeVar,
    Union,
    get_args,
    get_origin,
    get_type_hints,
)

from sqlalchemy import Column, Float, Integer, MetaData, String, Table
from sqlalchemy.ext.asyncio import AsyncConnection
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.ext.automap import automap_base
from sqlalchemy.future import select

from ._db_serializable import DbSerializable

T = TypeVar("T")


class Measures(DbSerializable, Protocol):
    """Protocol for measures."""

    table: Any  # TODO

    def __getitem__(self, key: str) -> Union[int, float, str]:
        """
        Get a measure.

        :param key: Name of the measure to get.
        """
        pass

    def __setitem__(self, key: str, value: Union[int, float, str]) -> None:
        """
        Set a measure.

        :param key: Name of the measure to set.
        :param value: New value of the measure.
        """
        pass


def make_measures(table_name: str) -> Callable[[Type[T]], Type[T]]:
    """
    Convert a class to a Measures type.

    The class should only have declared and/or defined variables of type [int, float, str] and no functions,
    similar to a dataclass.

    Example:
    @make_measures(table_name="my_measures")
    class MyMeasures:
        measure1: Optional[float] = None
        measure2: Optional[str] = "test"
    """

    def impl(cls: Type[T]) -> Type[T]:
        @dataclass
        class ColDescr:
            name: str
            xtype: Union[int, float, str]
            nullable: bool

        def make_column(name: str, xtype: Type[Any]) -> ColDescr:
            if get_origin(xtype) == Union:
                args = get_args(xtype)
                assert len(args) == 2, "types must be Type or Optional[Type]"
                if args[0] is type(None):
                    actual_type = args[1]
                elif args[1] is type(None):
                    actual_type = args[0]
                else:
                    assert False, "types must be Type or Optional[Type]"
                nullable = True
            else:
                actual_type = xtype
                nullable = False

            return ColDescr(name, actual_type, nullable)

        columns = [
            make_column(name, xtype) for name, xtype in get_type_hints(cls).items()
        ]

        metadata = MetaData()
        Table(
            table_name,
            metadata,
            Column(
                "id",
                Integer,
                nullable=False,
                primary_key=True,
                unique=True,
                autoincrement=True,
            ),
            *[
                Column(col.name, _sqlalchemy_type(col.xtype), nullable=col.nullable)
                for col in columns
            ],
        )
        db_base = automap_base(metadata=metadata)
        db_base.prepare()

        class MeasuresImpl(dataclass(cls, eq=False)):  # type: ignore # TODO
            __columns = [col.name for col in columns]
            __db_base = db_base
            table = db_base.classes[table_name]

            @classmethod
            async def prepare_db(cls, conn: AsyncConnection) -> None:
                await conn.run_sync(cls.__db_base.metadata.create_all)

            async def to_db(self, ses: AsyncSession) -> Column[Integer]:
                row = self.table(**{c: self[c] for c in self.__columns})
                ses.add(row)
                await ses.flush()
                return row.id  # type: ignore # TODO

            @classmethod
            async def from_db(
                cls, ses: AsyncSession, id: Column[Integer]
            ) -> MeasuresImpl:
                row = (
                    await ses.execute(select(cls.table).filter(cls.table.id == id))
                ).scalar_one_or_none()
                return cls(**{c: getattr(row, c) for c in cls.__columns})

            def __getitem__(self, key: str) -> Union[int, float, str, None]:
                assert key in self.__columns, "measure {key} does not exist"
                val = getattr(self, key)
                assert (
                    val is None
                    or type(val) == int
                    or type(val) == float
                    or type(val) == str
                )
                return val

            def __setitem__(
                self, key: str, value: Union[int, float, str, None]
            ) -> None:
                setattr(self, key, value)

        return MeasuresImpl

    return impl


def _sqlalchemy_type(type: Any) -> Union[Type[Integer], Type[Float], Type[String]]:
    if type == int:
        return Integer
    elif type == float:
        return Float
    elif type == String:
        return String
    else:
        raise ValueError()
