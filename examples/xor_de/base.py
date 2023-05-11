import sqlalchemy.orm as orm


class Base(orm.MappedAsDataclass, orm.DeclarativeBase):
    pass
