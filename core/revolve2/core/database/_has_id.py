import sqlalchemy.orm as orm


class HasId(orm.MappedAsDataclass):
    id: orm.Mapped[int] = orm.mapped_column(
        primary_key=True,
        unique=True,
        autoincrement=True,
        nullable=False,
        init=False,
    )
