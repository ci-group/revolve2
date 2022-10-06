from __future__ import annotations

from typing import Any, Generic, List, Type, TypeVar, Union, Optional

from sqlalchemy import Column, Float, Integer, String
from sqlalchemy.ext.asyncio import AsyncConnection
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.future import select
from sqlalchemy.orm import declarative_base

from ._serializable import Serializable

T = TypeVar("T", bound=Union[int, float, str, Serializable])


class SerializableList(Serializable, List[T], Generic[T]):
    """Interface for lists that can be serialized to a database."""

    item_table: Any  # TODO


def serializable_list_template(
    item_type: Type[T], table_name: str, value_column_name: str = "value"
) -> Type[SerializableList[T]]:
    """
    Create a SerializableList type using the provided generic parameters.

    :param item_type: Type of the contained items.
    :param table_name: Name of the corresponding table in the database.
    :param value_column_name: Name of value column in the database.
    :returns: The created SerializableList type.
    """

    def basic_type_or_serializable(type: Type[T]) -> bool:
        """
        Check whether the given type is a basic type or Serializable, or raises error if neither.

        :param type: The type to check.
        :returns: True when basic type, false when Serializable.
        :raises ValueError: When neither.
        """
        if type == int or type == float or type == str:
            return True
        elif issubclass(type, Serializable):
            return False
        else:
            assert False, "Type must be 'int', 'float', 'str' or 'Serializable'."

    def sqlalchemy_type(type: Any) -> Union[Type[Integer], Type[Float], Type[String]]:
        if type == int:
            return Integer
        elif type == float:
            return Float
        elif type == String:
            return String
        else:
            assert False, "Unsupported type."

    is_basic_type = basic_type_or_serializable(item_type)

    DbBase = declarative_base()

    class ListTable(DbBase):
        """Main table for the PopList."""

        __tablename__ = table_name

        id = Column(
            Integer,
            nullable=False,
            unique=True,
            autoincrement=True,
            primary_key=True,
        )

    if is_basic_type:
        dbtype = sqlalchemy_type(item_type)
    else:
        dbtype = Integer

    class ItemTable(DbBase):
        """Table for items in the PopList."""

        __tablename__ = f"{table_name}_item"

        id = Column(
            Integer,
            nullable=False,
            unique=True,
            autoincrement=True,
            primary_key=True,
        )
        list_id = Column(Integer, nullable=False, name=f"{table_name}_id")
        index = Column(
            Integer,
            nullable=False,
        )
        value = Column(dbtype, nullable=False, name=value_column_name)

    if is_basic_type:
        TBasicType = TypeVar("TBasicType", bound=Union[int, float, str])

        class SerializableListImplBasicType(SerializableList[TBasicType]):
            __db_base = DbBase
            __item_type = item_type

            table = ListTable
            item_table = ItemTable

            @classmethod
            async def prepare_db(cls, conn: AsyncConnection) -> None:
                await conn.run_sync(cls.__db_base.metadata.create_all)

            async def to_db(
                self,
                ses: AsyncSession,
            ) -> int:
                dblist = self.table()
                ses.add(dblist)
                await ses.flush()
                assert dblist.id is not None

                items = [
                    self.item_table(list_id=dblist.id, index=i, value=v)  # type: ignore # TODO
                    for i, v in enumerate(self)
                ]
                ses.add_all(items)

                return int(dblist.id)

            @classmethod
            async def from_db(
                cls, ses: AsyncSession, id: int
            ) -> Optional[SerializableListImplBasicType[TBasicType]]:
                row = (
                    await ses.execute(select(cls.table).filter(cls.table.id == id))
                ).scalar_one_or_none()

                if row is None:
                    return None

                rows = (
                    await ses.execute(
                        select(cls.item_table)
                        .filter(cls.item_table.list_id == id)
                        .order_by(cls.item_table.index)
                    )
                ).scalars()

                return cls([cls.__item_type(row.value) for row in rows])

        return SerializableListImplBasicType
    else:
        TSerializable = TypeVar("TSerializable", bound=Serializable)

        class SerializableListSerializable(SerializableList[TSerializable]):
            __db_base = DbBase
            __item_type = item_type

            table = ListTable
            item_table = ItemTable

            @classmethod
            async def prepare_db(cls, conn: AsyncConnection) -> None:
                await cls.__item_type.prepare_db(conn)
                await conn.run_sync(cls.__db_base.metadata.create_all)

            async def to_db(
                self,
                ses: AsyncSession,
            ) -> int:
                dblist = self.table()
                ses.add(dblist)
                await ses.flush()
                assert dblist.id is not None

                values = [await i.to_db(ses) for i in self]

                items = [
                    self.item_table(list_id=dblist.id, index=i, value=v)  # type: ignore # TODO
                    for i, v in enumerate(values)
                ]
                ses.add_all(items)

                return int(dblist.id)

            @classmethod
            async def from_db(
                cls, ses: AsyncSession, id: int
            ) -> Optional[SerializableListSerializable[TSerializable]]:
                row = (
                    await ses.execute(select(cls.table).filter(cls.table.id == id))
                ).scalar_one_or_none()

                if row is None:
                    return None

                rows = (
                    await ses.execute(
                        select(cls.item_table)
                        .filter(cls.item_table.list_id == id)
                        .order_by(cls.item_table.index)
                    )
                ).scalars()

                return cls(
                    [await cls.__item_type.from_db(ses, row.value) for row in rows]
                )

        return SerializableListSerializable
