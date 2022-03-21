from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Integer, Column, String, Float, PickleType

DbBase = declarative_base()


class DbOptimizerState(DbBase):
    __tablename__ = "optimizer_state"

    process_id = Column(
        Integer,
        nullable=False,
        primary_key=True,
    )
    generation_index = Column(Integer, nullable=False, primary_key=True)
    rng = Column(PickleType, nullable=False)
