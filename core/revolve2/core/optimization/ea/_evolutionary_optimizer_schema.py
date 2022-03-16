from sqlalchemy.ext.declarative import declarative_base

DbBase = declarative_base()


class DbEvolutionaryOptimizer(DbBase):
    from sqlalchemy import Integer, Column, String

    __tablename__ = "evolutionary_optimizer"

    id = Column(
        Integer,
        nullable=False,
        unique=True,
        autoincrement=True,
        primary_key=True,
    )
    process_id = Column(Integer, nullable=False, unique=True)
    offspring_size = Column(Integer, nullable=False)
    genotype_type = Column(String, nullable=False)
    fitness_type = Column(String, nullable=False)


class DbEvolutionaryOptimizerState(DbBase):
    from sqlalchemy import Integer, Column

    __tablename__ = "evolutionary_optimizer_state"

    evolutionary_optimizer_id = Column(Integer, nullable=False, primary_key=True)
    generation_index = Column(Integer, nullable=False, primary_key=True)
    processid_state = Column(Integer, nullable=False)


class DbEvolutionaryOptimizerGeneration(DbBase):
    from sqlalchemy import Integer, Column

    __tablename__ = "evolutionary_optimizer_generation"

    evolutionary_optimizer_id = Column(Integer, nullable=False, primary_key=True)
    generation_index = Column(Integer, nullable=False, primary_key=True)
    individual_index = Column(Integer, nullable=False, primary_key=True)
    individual_id = Column(Integer, nullable=False)


class DbEvolutionaryOptimizerIndividual(DbBase):
    from sqlalchemy import Integer, Column

    __tablename__ = "evolutionary_optimizer_individual"

    evolutionary_optimizer_id = Column(Integer, nullable=False, primary_key=True)
    individual_id = Column(Integer, nullable=False, primary_key=True)
    genotype_id = Column(Integer, nullable=False)
    fitness_id = Column(Integer, nullable=True)
    # TODO parents
