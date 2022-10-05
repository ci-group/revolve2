from __future__ import annotations
from abc import abstractmethod

from typing import List, Generic, Type, TypeVar, Tuple

from .._measures import Measures
from .._serializable import Serializable, make_serializable
from .._serializable_list import SerializableList, serializable_list_template

TGenotype = TypeVar("TGenotype", bound=Serializable)
TMeasures = TypeVar("TMeasures", bound=Measures)


class Individual(Serializable, Generic[TGenotype, TMeasures]):
    """An individual, consisting of a genotype and its measures."""

    genotype: TGenotype
    measures: TMeasures


class PopList(
    SerializableList[Individual[TGenotype, TMeasures]], Generic[TGenotype, TMeasures]
):
    """Interface for the generic PopList class."""

    @classmethod
    @abstractmethod
    def from_existing_populations(
        cls,
        populations: List[PopList[TGenotype, TMeasures]],
        selections: List[List[int]],
        copied_measures: List[str],
    ) -> PopList[TGenotype, TMeasures]:
        """
        Create a population from a set of existing populations using a provided selection from each population and copying the provided measures.

        :param populations: The populations to combine.
        :param selection: The individuals to select from each population.
        :param copied_measures: The measures to copy.
        :returns: The created population.
        """


def pop_list_template(
    table_name: str, genotype_type: Type[TGenotype], measures_type: Type[TMeasures]
) -> Tuple[Type[PopList[TGenotype, TMeasures]], Type[Individual[TGenotype, TMeasures]]]:
    """
    Create a PopList type using the provided generic parameters.

    :param genotype_type: Type of the genotype.
    :param measures_type: Type of the measures.
    :returns: The created PopList type.
    """

    IndividualImpl = make_serializable(f"{table_name}_individual")(
        type(
            "IndividualImpl",
            (),
            {"__annotations__": {"genotype": genotype_type, "measures": measures_type}},
        )
    )

    IndividualListImpl = serializable_list_template(
        IndividualImpl, table_name, "individual"
    )

    class PopListImpl(IndividualListImpl, PopList[TGenotype, TMeasures]):
        """A population stored as a list of individuals."""

        __individual_type = IndividualImpl

        @classmethod
        def from_existing_populations(
            cls,
            populations: List[PopList[TGenotype, TMeasures]],
            selections: List[List[int]],
            copied_measures: List[str],
        ) -> PopList[TGenotype, TMeasures]:
            new_individuals: List[IndividualImpl[TGenotype, TMeasures]] = []
            for pop, selection in zip(populations, selections):
                for i in selection:
                    new_ind = cls.__individual_type(
                        pop[i].genotype, type(pop[i].measures)()
                    )
                    for measure in copied_measures:
                        new_ind.measures[measure] = pop[i].measures[measure]
                    new_individuals.append(new_ind)

            return PopListImpl(new_individuals)

    return PopListImpl, IndividualImpl
