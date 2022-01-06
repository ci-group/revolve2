from typing import Optional

from sqlalchemy import Column, Integer, String
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql.expression import null

Base = declarative_base()


class DbNode(Base):
    __tablename__ = "nodes"
    id = Column(
        Integer, nullable=False, unique=True, autoincrement=True, primary_key=True
    )
    type = Column(Integer, nullable=False)  # 0: uninitialized, 1: object, 2: list
    object = Column(String, nullable=True)

    def __init__(self, type: int, object: Optional[str], id: int = None) -> None:
        self.id = id
        self.type = type
        self.object = object


class DbListItem(Base):
    __tablename__ = "list_items"
    id = Column(
        Integer, nullable=False, unique=True, autoincrement=True, primary_key=True
    )
    index = Column(Integer, nullable=False)
    list_node_id = Column(Integer, nullable=False)
    child_node_id = Column(Integer, nullable=False)

    def __init__(self, index: int, list_node_id: int, child_node_id: int) -> None:
        self.index = index
        self.list_node_id = list_node_id
        self.child_node_id = child_node_id
