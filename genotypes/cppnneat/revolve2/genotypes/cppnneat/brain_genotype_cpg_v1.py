from revolve2.core.modular_robot import Body as ModularRobotBody
from revolve2.core.modular_robot import Brain as ModularRobotBrain
from revolve2.core.optimization.ea.modular_robot import BrainGenotype

from .bodybrain_base import BodybrainBase
from .brain_cpg_v1 import BrainCpgV1


class BrainGenotypeCpgV1(BrainGenotype, BodybrainBase):
    def develop(self, body: ModularRobotBody) -> ModularRobotBrain:
        return BrainCpgV1(self._genotype)
