"""Base class."""

import sqlalchemy.orm as orm


class Base(orm.MappedAsDataclass, orm.DeclarativeBase):
    """Base class for all SQLAlchemy models in this example."""

    pass
