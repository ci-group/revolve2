from sqlalchemy import Column, Float, Integer, PickleType, String
from sqlalchemy.ext.declarative import declarative_base

DbBase = declarative_base()


class DbOptimizerState(DbBase):
    __tablename__ = "optimizer"

    process_id = Column(
        Integer,
        nullable=False,
        primary_key=True,
    )
    generation_index = Column(Integer, nullable=False, primary_key=True)
    rng = Column(PickleType, nullable=False)
    innov_db_body = Column(String, nullable=False)
    innov_db_brain = Column(String, nullable=False)
    simulation_time = Column(Integer, nullable=False)
    sampling_frequency = Column(Float, nullable=False)
    control_frequency = Column(Float, nullable=False)
    num_generations = Column(Integer, nullable=False)
