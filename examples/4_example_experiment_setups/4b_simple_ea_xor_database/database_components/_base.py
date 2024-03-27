"""Base class."""

import sqlalchemy.orm as orm


class Base(orm.MappedAsDataclass, orm.DeclarativeBase):
    """
    Base class for all SQLAlchemy models in this example.

    All ORM classes should inherit from this.
    This is simply how SQLAlchemy is designed.
    We can then use this base class to create the structure of our new (empty) database,
    automatically knowing all classes that inherited from the base.
    """

    pass
