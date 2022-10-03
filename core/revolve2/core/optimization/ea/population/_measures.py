from multiprocessing.sharedctypes import Value
from typing import Any, Dict, List, Tuple, Union
from sqlalchemy import Table, MetaData, Column, Integer, Float, String
from sqlalchemy.ext.asyncio import AsyncConnection
from sqlalchemy.ext.automap import automap_base

Measures = Dict[str, Any]


def _sqlalchemy_type(type):
    if type == int:
        return Integer
    elif type == float:
        return Float
    elif type == String:
        return String
    else:
        return ValueError()


def make_measures_class(
    name: str, table_name: str, columns: List[Tuple[str, Union[int, float, str]]]
) -> Any:
    metadata = MetaData()
    table = Table(
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
        *[Column(n, _sqlalchemy_type(t), nullable=True) for n, t in columns],
    )
    db_base = automap_base(metadata=metadata)
    db_base.prepare()
    table_orm = db_base.classes[table_name]

    @classmethod
    async def prepare_db(cls, conn: AsyncConnection) -> None:
        await conn.run_sync(cls.__db_base.metadata.create_all)

    def __getitem__(self, key):
        assert key in self.__columns, "measure {key} does not exist"
        return getattr(self, key)

    def __setitem__(self, key, value):
        setattr(self, key, value)

    def to_row(self):
        return self.table(**{c: self[c] for c in self.__columns})

    return type(
        name,
        (),
        {
            "__db_base": db_base,
            "table": table_orm,
            "prepare_db": prepare_db,
            "to_row": to_row,
            "__columns": [n for n, _ in columns],
            "__getitem__": __getitem__,
            "__setitem__": __setitem__,
            **{n: None for n, _ in columns},
            **{"__annotations__": {n: t for n, t in columns}},
        },
    )
