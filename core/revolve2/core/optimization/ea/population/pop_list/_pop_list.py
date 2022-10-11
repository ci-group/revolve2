from __future__ import annotations

from typing import Any, Dict, List, Optional, Type, TypeVar, get_args

from revolve2.core.database import Serializable
from sqlalchemy import Column, Integer
from sqlalchemy.ext.asyncio import AsyncConnection
from sqlalchemy.ext.asyncio.session import AsyncSession
from sqlalchemy.future import select
from sqlalchemy.orm import declarative_base

from .._individual import Individual
from .._serializable_measures import SerializableMeasures

TGenotype = TypeVar("TGenotype", bound=Serializable)
TMeasures = TypeVar("TMeasures", bound=SerializableMeasures)


class PopList(List[Individual[TGenotype, TMeasures]], Serializable):
    """A population consisting of an ordered list of individuals."""

    __db_base: type  # TODO proper type
    __genotype_type: Type[TGenotype]
    __measures_type: Type[TMeasures]
    item_table: Any

    @classmethod
    def __init_subclass__(cls, /, table_name: str, **kwargs: Dict[str, Any]) -> None:
        """
        Initialize this object.

        :param table_name: Prefix of all tables in the database.
        :param kwargs: Other arguments not specific to this class.
        """
        super().__init_subclass__(**kwargs)

        assert len(cls.__orig_bases__) == 1  # type: ignore # TODO
        cls.__genotype_type, cls.__measures_type = get_args(cls.__orig_bases__[0])  # type: ignore # TODO

        cls.__db_base = declarative_base()

        class ListTable(cls.__db_base):  # type: ignore # Mypy does not understand this dynamic base class.
            """Main table for the PopList."""

            __tablename__ = table_name

            id = Column(
                Integer,
                nullable=False,
                unique=True,
                autoincrement=True,
                primary_key=True,
            )

        class ItemTable(cls.__db_base):  # type: ignore # Mypy does not understand this dynamic base class.
            """Table for items in the PopList."""

            __tablename__ = f"{table_name}_individual"

            id = Column(
                Integer,
                nullable=False,
                unique=True,
                autoincrement=True,
                primary_key=True,
            )
            list_id = Column(Integer, nullable=False, name=f"{table_name}_id")
            index = Column(
                Integer,
                nullable=False,
            )
            genotype = Column(Integer, nullable=False)
            measures = Column(Integer, nullable=False)

        cls.table = ListTable
        cls.item_table = ItemTable

    @classmethod
    async def prepare_db(cls, conn: AsyncConnection) -> None:
        """
        Set up the database, creating tables.

        :param conn: Connection to the database.
        """
        await cls.__genotype_type.prepare_db(conn)
        await cls.__measures_type.prepare_db(conn)
        await conn.run_sync(cls.__db_base.metadata.create_all)  # type: ignore # TODO

    @classmethod
    async def to_db_multiple(
        cls, ses: AsyncSession, objects: List[PopList[TGenotype, TMeasures]]
    ) -> List[int]:
        """
        Serialize multiple objects to a database.

        :param ses: Database session.
        :param objects: The objects to serialize.
        :returns: Ids of the objects in the database.
        """
        dblists = [cls.table() for _ in objects]
        ses.add_all(dblists)
        await ses.flush()
        db_ids: List[int] = [int(l.id) for l in dblists]

        combined_genotypes = [i.genotype for o in objects for i in o]
        combined_measures = [i.measures for o in objects for i in o]
        combined_genotype_ids = await cls.__genotype_type.to_db_multiple(
            ses, combined_genotypes
        )
        combined_value_ids = await cls.__measures_type.to_db_multiple(
            ses, combined_measures
        )

        items = []
        i = 0
        for object, db_id in zip(objects, db_ids):
            for index, (genotype_id, value_id) in enumerate(
                zip(
                    combined_genotype_ids[i : i + len(object)],
                    combined_value_ids[i : i + len(object)],
                )
            ):
                items.append(
                    cls.item_table(
                        list_id=db_id,
                        index=index,
                        genotype=genotype_id,
                        measures=value_id,
                    )
                )
            i += len(object)

        ses.add_all(items)

        return db_ids

    @classmethod
    async def from_db(
        cls, ses: AsyncSession, id: int
    ) -> Optional[PopList[TGenotype, TMeasures]]:  # TODO return type should be Self
        """
        Deserialize this object from a database.

        If id does not exist, returns None.

        :param ses: Database session.
        :param id: Id of the object in the database.
        :returns: The deserialized object or None is id does not exist.
        """
        row = (
            await ses.execute(select(cls.table).filter(cls.table.id == id))
        ).scalar_one_or_none()

        if row is None:
            return None

        rows = (
            await ses.execute(
                select(cls.item_table)
                .filter(cls.item_table.list_id == id)
                .order_by(cls.item_table.index)
            )
        ).scalars()

        return cls(
            [
                Individual(
                    await cls.__genotype_type.from_db(ses, row.genotype),
                    await cls.__measures_type.from_db(ses, row.measures),
                )
                for row in rows
            ]
        )

    @classmethod
    def from_existing_populations(
        cls,
        populations: List[PopList[TGenotype, TMeasures]],
        selections: List[List[int]],
        copied_measures: List[str],
    ) -> PopList[TGenotype, TMeasures]:  # TODO return type should be Self
        """
        Create a population from a set of existing populations using a provided selection from each population and copying the provided measures.

        :param populations: The populations to combine.
        :param selections: The individuals to select from each population.
        :param copied_measures: The measures to copy.
        :returns: The created population.
        """
        new_individuals: List[Individual[TGenotype, TMeasures]] = []
        for pop, selection in zip(populations, selections):
            for i in selection:
                new_ind = Individual(pop[i].genotype, type(pop[i].measures)())
                for measure in copied_measures:
                    new_ind.measures[measure] = pop[i].measures[measure]
                new_individuals.append(new_ind)

        return cls(new_individuals)
