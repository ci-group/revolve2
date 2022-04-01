from sqlalchemy import Column, Integer, String
from sqlalchemy.ext.declarative import declarative_base

DbBase = declarative_base()


class DbEAOptimizer(DbBase):
    __tablename__ = "ea_optimizer"

    id = Column(
        Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )
    process_id = Column(Integer, nullable=False, unique=True)
    offspring_size = Column(Integer, nullable=False)
    genotype_table = Column(String, nullable=False)
    fitness_table = Column(String, nullable=False)


class DbEAOptimizerState(DbBase):
    __tablename__ = "ea_optimizer_state"

    ea_optimizer_id = Column(Integer, nullable=False, primary_key=True)
    generation_index = Column(Integer, nullable=False, primary_key=True)
    processid_state = Column(Integer, nullable=False)


class DbEAOptimizerGeneration(DbBase):
    __tablename__ = "ea_optimizer_generation"

    ea_optimizer_id = Column(Integer, nullable=False, primary_key=True)
    generation_index = Column(Integer, nullable=False, primary_key=True)
    individual_index = Column(Integer, nullable=False, primary_key=True)
    individual_id = Column(Integer, nullable=False)


class DbEAOptimizerIndividual(DbBase):
    __tablename__ = "ea_optimizer_individual"

    ea_optimizer_id = Column(Integer, nullable=False, primary_key=True)
    individual_id = Column(Integer, nullable=False, primary_key=True)
    genotype_id = Column(Integer, nullable=False)
    fitness_id = Column(Integer, nullable=True)


class DbEAOptimizerParent(DbBase):
    __tablename__ = "ea_optimizer_parent"

    ea_optimizer_id = Column(Integer, nullable=False, primary_key=True)
    child_individual_id = Column(Integer, nullable=False, primary_key=True)
    parent_individual_id = Column(Integer, nullable=False, primary_key=True)
