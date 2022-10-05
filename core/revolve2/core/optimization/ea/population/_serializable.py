from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import (
    Any,
    Callable,
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

SerializableSelf = TypeVar("SerializableSelf", bound="Serializable")


class Serializable(ABC):
    """Interface for classes that can be serialized to a database."""

    table: Any  # TODO

    @classmethod
    @abstractmethod
    async def prepare_db(cls, conn: AsyncConnection) -> None:
        """
        Set up the database, creating tables.

        :param conn: Connection to the database.
        """
        pass

    @abstractmethod
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
    @abstractmethod
    async def from_db(
        cls: Type[SerializableSelf], ses: AsyncSession, id: int
    ) -> SerializableSelf:
        """
        Deserialize this object from a database.

        :param ses: Database session.
        :param id: Id of the object in the database.
        """
        pass


SerializableTypes = Union[Type[int], Type[float], Type[str], Type[Serializable]]

T = TypeVar("T", bound=SerializableTypes)


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

    def impl(cls: Type[T]) -> Type[T]:  # TODO should be Intersection[T, Serializable]
        @dataclass
        class ColDescr:
            name: str
            ctype: SerializableTypes
            nullable: bool

        def make_column(name: str, ctype: Any) -> ColDescr:
            if get_origin(ctype) == Union:
                args = get_args(ctype)
                assert (
                    len(args) == 2
                ), "All member types must be one of 'int', 'float', 'str', 'Serializable', 'Optional[int]', 'Optional[float]', 'Optional[str]', 'Optional[Serializable]'"
                if args[0] is type(None):
                    actual_type = args[1]
                elif args[1] is type(None):
                    actual_type = args[0]
                else:
                    assert (
                        False
                    ), "All member types must be one of 'int', 'float', 'str', 'Serializable', 'Optional[int]', 'Optional[float]', 'Optional[str]', 'Optional[Serializable]'"
                nullable = True
            else:
                actual_type = ctype
                nullable = False

            assert (
                issubclass(actual_type, Serializable)
                or actual_type == int
                or actual_type == float
                or actual_type == str
            ), "All member types must be one of 'int', 'float', 'str', 'Serializable', 'Optional[int]', 'Optional[float]', 'Optional[str]', 'Optional[Serializable]'"

            return ColDescr(name, actual_type, nullable)

        def _sqlalchemy_type(
            ctype: Any,
        ) -> Union[Type[Integer], Type[Float], Type[String]]:
            if issubclass(ctype, Serializable):
                return Integer
            elif ctype == int:
                return Integer
            elif ctype == float:
                return Float
            elif ctype == String:
                return String
            else:
                assert False, "Unsupported type."

        columns = [
            make_column(name, ctype) for name, ctype in get_type_hints(cls).items()
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
                Column(col.name, _sqlalchemy_type(col.ctype), nullable=col.nullable)
                for col in columns
            ],
        )
        db_base = automap_base(metadata=metadata)
        db_base.prepare()

        class SerializableImpl(dataclass(cls, eq=False), Serializable):  # type: ignore # TODO
            __column_names = [col.name for col in columns]
            __column_types = [col.ctype for col in columns]
            __db_base = db_base
            table = db_base.classes[table_name]

            async def __to_db_if_serializable(
                self, ses: AsyncSession, column_index: int
            ) -> Union[int, float, str]:
                value = getattr(self, self.__column_names[column_index])
                if issubclass(self.__column_types[column_index], Serializable):
                    assert isinstance(value, Serializable)
                    return await value.to_db(ses)
                else:
                    assert isinstance(value, self.__column_types[column_index])
                    return value  # type: ignore # TODO

            @classmethod
            async def __from_db_if_serializable(
                cls, ses: AsyncSession, column_index: int, value: Union[int, float, str]
            ) -> Union[int, float, str, Serializable]:
                if issubclass(cls.__column_types[column_index], Serializable):
                    return await cls.__column_types[column_index].from_db(ses, value)  # type: ignore # TODO
                else:
                    return value

            @classmethod
            async def prepare_db(cls, conn: AsyncConnection) -> None:
                for ctype in cls.__column_types:
                    if issubclass(ctype, Serializable):
                        await ctype.prepare_db(conn)
                await conn.run_sync(cls.__db_base.metadata.create_all)

            async def to_db(self, ses: AsyncSession) -> int:
                row = self.table(
                    **{
                        self.__column_names[i]: await self.__to_db_if_serializable(
                            ses, i
                        )
                        for i in range(len(self.__column_names))
                    }
                )
                ses.add(row)
                await ses.flush()
                return int(row.id)

            @classmethod
            async def from_db(cls, ses: AsyncSession, id: int) -> SerializableImpl:
                row = (
                    await ses.execute(select(cls.table).filter(cls.table.id == id))
                ).scalar_one_or_none()
                return cls(
                    **{
                        cls.__column_names[i]: await cls.__from_db_if_serializable(
                            ses, i, getattr(row, cls.__column_names[i])
                        )
                        for i in range(len(cls.__column_names))
                    }
                )

        return SerializableImpl

    return impl
