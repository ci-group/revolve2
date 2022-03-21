from sqlalchemy.ext.declarative import declarative_base
import sqlalchemy

DbBase = declarative_base()


class DbFitnessFloat(DbBase):
    __tablename__ = "fitness"

    id = sqlalchemy.Column(
        sqlalchemy.Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )
    fitness = sqlalchemy.Column(sqlalchemy.Integer, nullable=False)
