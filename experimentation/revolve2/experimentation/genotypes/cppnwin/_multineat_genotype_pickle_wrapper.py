from dataclasses import dataclass
from typing import cast

import multineat


@dataclass
class MultineatGenotypePickleWrapper:
    """A wrapper about multineat.Genome that provides pickling."""

    genotype: multineat.Genome

    def __getstate__(self) -> str:
        """
        Convert the genotype to a string, serializing it.

        :returns: The string.
        """
        return cast(str, self.genotype.Serialize()).replace(" ", "")

    def __setstate__(self, serialized_genotype: str) -> None:
        """
        Convert a string obtained through __getstate__ to a genotype and set it as the genotype.

        :param serialized_genotype: The string to convert.
        """
        genotype = multineat.Genome()
        genotype.Deserialize(serialized_genotype)
        self.genotype = genotype
