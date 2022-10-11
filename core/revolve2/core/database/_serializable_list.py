from __future__ import annotations

from typing import Any, Dict, List, Optional, Type, TypeVar, Union, get_args

from sqlalchemy import Column, Float, Integer, String
from sqlalchemy.ext.asyncio import AsyncConnection
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.future import select
from sqlalchemy.orm import declarative_base

from ._serializable import Serializable

T = TypeVar("T", bound=Union[int, float, str, Serializable])


class SerializableList(List[T], Serializable):
    """A python list that can be serialized to the database."""

    __db_base: type  # TODO proper type
    __item_type: Type[T]
    __is_basic_type: bool
    item_table: Any

    @classmethod
    def __init_subclass__(
        cls, /, table_name: str, value_column_name: str, **kwargs: Dict[str, Any]
    ) -> None:
        """
        Initialize this object.

        :param table_name: Prefix of all tables in the database.
        :param value_column_name: Name of the value column in the database table.
        :param kwargs: Other arguments not specific to this class.
        """
        super().__init_subclass__(**kwargs)

        assert len(cls.__orig_bases__) == 1  # type: ignore # TODO
        cls.__item_type = get_args(cls.__orig_bases__[0])[0]  # type: ignore # TODO

        dbtype: Union[Type[Integer], Type[Float], Type[String]]

        if cls.__item_type == int:
            cls.__is_basic_type = True
            dbtype = Integer
        elif cls.__item_type == float:
            cls.__is_basic_type = True
            dbtype = Float
        elif cls.__item_type == String:
            cls.__is_basic_type = True
            dbtype = String
        elif issubclass(cls.__item_type, Serializable):
            cls.__is_basic_type = False
            dbtype = Integer
        else:
            assert False, "Type must be 'int', 'float', 'str' or 'Serializable'."

        cls.__db_base = declarative_base()

        class ListTable(cls.__db_base):  # type: ignore # Mypy does not understand this dynamic base class.
            """Main table for the PopList."""

            __tablename__ = table_name

            id = Column(
                Integer,
                nullable=False,
                unique=True,
                autoincrement=True,
                primary_key=True,
            )

        class ItemTable(cls.__db_base):  # type: ignore # Mypy does not understand this dynamic base class.
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

        cls.table = ListTable
        cls.item_table = ItemTable

    @classmethod
    async def prepare_db(cls, conn: AsyncConnection) -> None:
        """
        Set up the database, creating tables.

        :param conn: Connection to the database.
        """
        await conn.run_sync(cls.__db_base.metadata.create_all)  # type: ignore # TODO

    @classmethod
    async def to_db_multiple(
        cls, ses: AsyncSession, objects: List[SerializableList[T]]
    ) -> List[int]:
        """
        Serialize multiple objects to a database.

        :param ses: Database session.
        :param objects: The objects to serialize.
        :returns: Ids of the objects in the database.
        """
        dblists = [cls.table() for _ in objects]
        ses.add_all(dblists)
        await ses.flush()
        db_ids: List[int] = [int(l.id) for l in dblists]

        if cls.__is_basic_type:
            combined_values = [i for o in objects for i in o]
        else:
            combined_unserialized_values = [i for o in objects for i in o]
            combined_values = await cls.__item_type.to_db_multiple(
                ses, combined_unserialized_values
            )

        items = []
        i = 0
        for object, db_id in zip(objects, db_ids):
            for index, value in enumerate(
                combined_values[i : i + len(object)],
            ):
                items.append(cls.item_table(list_id=db_id, index=index, value=value))
            i += len(object)

        ses.add_all(items)

        return db_ids

    @classmethod
    async def from_db(cls, ses: AsyncSession, id: int) -> Optional[SerializableList[T]]:
        """
        Deserialize this object from a database.

        If id does not exist, returns None.

        :param ses: Database session.
        :param id: Id of the object in the database.
        :returns: The deserialized object or None is id does not exist.
        """
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

        if cls.__is_basic_type:
            return cls([cls.__item_type(row.value) for row in rows])
        else:
            return cls([await cls.__item_type.from_db(ses, row.value) for row in rows])
