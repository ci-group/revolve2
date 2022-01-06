from typing import Generic, TypeVar

from revolve2.core.modular_robot import ModularRobot

from .body_genotype import BodyGenotype
from .brain_genotype import BrainGenotype

BodyGenotypeImpl = TypeVar("BodyGenotypeImpl", bound=BodyGenotype)
BrainGenotypeImpl = TypeVar("BrainGenotypeImpl", bound=BrainGenotype)


class BodybrainGenotype(Generic[BodyGenotypeImpl, BrainGenotypeImpl]):
    _body_genotype: BodyGenotypeImpl
    _brain_genotype: BrainGenotypeImpl

    def __init__(
        self, body_genotype: BodyGenotypeImpl, brain_genotype: BrainGenotypeImpl
    ):
        self._body_genotype = body_genotype
        self._brain_genotype = brain_genotype

    def develop(self) -> ModularRobot:
        body = self._body_genotype.develop()
        brain = self._brain_genotype.develop(body)
        return ModularRobot(body, brain)
