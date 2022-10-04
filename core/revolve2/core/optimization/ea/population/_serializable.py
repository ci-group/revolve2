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

T = TypeVar("T")

SerializableSelf = TypeVar("SerializableSelf", bound="Serializable")


class Serializable(Protocol):
    """Protocol for classes that can be serialized to a database."""

    table: Any  # TODO

    @classmethod
    async def prepare_db(cls, conn: AsyncConnection) -> None:
        """
        Set up the database, creating tables.

        :param conn: Connection to the database.
        """
        pass

    async def to_db(
        self: SerializableSelf,
        ses: AsyncSession,
    ) -> int:
        """
        Serialize this object to a database.

        :param ses: Database session.
        :returns: Id of the object in the database.
        """
        pass

    @classmethod
    async def from_db(
        cls: Type[SerializableSelf], ses: AsyncSession, id: int
    ) -> SerializableSelf:
        """
        Deserialize this object from a database.

        :param ses: Database session.
        :param id: Id of the object in the database.
        """
        pass


def make_serializable(table_name: str) -> Callable[[Type[T]], Type[T]]:
    """
    Make a class Serializable.

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

        class SerializableImpl(dataclass(cls, eq=False), Serializable):  # type: ignore # TODO
            __columns = [col.name for col in columns]
            __db_base = db_base
            table = db_base.classes[table_name]

            @classmethod
            async def prepare_db(cls, conn: AsyncConnection) -> None:
                await conn.run_sync(cls.__db_base.metadata.create_all)

            async def to_db(self, ses: AsyncSession) -> int:
                row = self.table(**{c: self[c] for c in self.__columns})
                ses.add(row)
                await ses.flush()
                return row.id  # type: ignore # TODO

            @classmethod
            async def from_db(cls, ses: AsyncSession, id: int) -> SerializableImpl:
                row = (
                    await ses.execute(select(cls.table).filter(cls.table.id == id))
                ).scalar_one_or_none()
                return cls(**{c: getattr(row, c) for c in cls.__columns})

        return SerializableImpl

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
