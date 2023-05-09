import sqlalchemy
import sqlalchemy.orm as orm
from base import Base
from revolve2.core.database import HasId
from revolve2.core.optimization.ea.parameters import Parameters as GenericParameters


class Parameters(Base, HasId, GenericParameters):
    __tablename__ = "parameters"

    generation_id: orm.Mapped[int] = orm.mapped_column(
        sqlalchemy.ForeignKey("generation.id"), nullable=False, init=False
    )
    generation_index: orm.Mapped[int] = orm.mapped_column(nullable=False, init=False)
    fitness: orm.Mapped[float] = orm.mapped_column(nullable=False, default=0.0)
