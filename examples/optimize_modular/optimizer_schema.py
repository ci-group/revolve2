from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import Integer, Column, String, Float, PickleType

DbBase = declarative_base()


class DbOptimizer(DbBase):
    __tablename__ = "optimizer"

    process_id = Column(
        Integer,
        nullable=False,
        unique=True,
        primary_key=True,
    )
    rng = Column(PickleType, nullable=False)
    innov_db_body = Column(String, nullable=False)
    innov_db_brain = Column(String, nullable=False)
    simulation_time = Column(Integer, nullable=False)
    sampling_frequency = Column(Float, nullable=False)
    control_frequency = Column(Float, nullable=False)
    num_generations = Column(Integer, nullable=False)
