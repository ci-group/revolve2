"""Database schema for CPPNWIN genotype."""

import sqlalchemy
from sqlalchemy.ext.declarative import declarative_base

DbBase = declarative_base()


class DbGenotype(DbBase):
    """Stores serialized multineat genomes."""

    __tablename__ = "cppnwin_genotype"

    id = sqlalchemy.Column(
        sqlalchemy.Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )
    serialized_multineat_genome = sqlalchemy.Column(sqlalchemy.String, nullable=False)
