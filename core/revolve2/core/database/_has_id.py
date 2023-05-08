import sqlalchemy.orm as orm


class HasId(orm.MappedAsDataclass):
    """An SQLAlchemy mixin that provides an id column."""

    id: orm.Mapped[int] = orm.mapped_column(
        primary_key=True,
        unique=True,
        autoincrement=True,
        nullable=False,
        init=False,
    )
