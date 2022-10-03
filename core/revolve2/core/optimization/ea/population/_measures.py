from typing import Union, TypeVar, Type, get_origin, get_type_hints, get_args, Callable
from sqlalchemy import Table, MetaData, Column, Integer, Float, String
from sqlalchemy.ext.asyncio import AsyncConnection
from sqlalchemy.ext.automap import automap_base
from dataclasses import dataclass

T = TypeVar("T")


def make_measures(table_name: str) -> Callable[[Type[T]], Type[T]]:
    def impl(cls: Type[T]) -> Type[T]:
        @dataclass
        class ColDescr:
            name: str
            xtype: Union[int, float, str]
            nullable: bool

        def make_column(name, xtype) -> ColDescr:
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

        class Measures(dataclass(cls, eq=False)):
            __columns = [col.name for col in columns]
            __db_base = db_base
            table = db_base.classes[table_name]

            @classmethod
            async def prepare_db(cls, conn: AsyncConnection) -> None:
                await conn.run_sync(cls.__db_base.metadata.create_all)

            def to_row(self):
                return self.table(**{c: self[c] for c in self.__columns})

            def __getitem__(self, key):
                assert key in self.__columns, "measure {key} does not exist"
                return getattr(self, key)

            def __setitem__(self, key, value):
                setattr(self, key, value)

        return Measures

    return impl


def _sqlalchemy_type(type):
    if type == int:
        return Integer
    elif type == float:
        return Float
    elif type == String:
        return String
    else:
        return ValueError()
