import multineat
from revolve2.core.database.serialize import Serializable
from revolve2.core.database.view import AnyView
from revolve2.core.modular_robot import Body as ModularRobotBody
from revolve2.core.modular_robot import Brain as ModularRobotBrain
from revolve2.core.optimization.ea.modular_robot import BrainGenotype

from .bodybrain_base import BodybrainBase
from .brain_cpg_v1 import BrainCpgV1


class BrainGenotypeCpgV1(BrainGenotype, BodybrainBase["BrainGenotypeCpgV1"]):
    @classmethod
    def random(
        cls,
        innov_db: multineat.InnovationDatabase,
        rng: multineat.RNG,
        multineat_params: multineat.Parameters,
        output_activation_func: multineat.ActivationFunction,
        num_initial_mutations: int,
    ) -> BodybrainBase:
        return super(BrainGenotypeCpgV1, cls).random(
            innov_db,
            rng,
            multineat_params,
            output_activation_func,
            7,  # bias(always 1), x1, y1, z1, x2, y2, z2
            1,  # weight
            num_initial_mutations,
        )

    def develop(self, body: ModularRobotBody) -> ModularRobotBrain:
        return BrainCpgV1(self._genotype)

    def to_database(self, db_view: AnyView) -> None:
        db_view.string = self._genotype.Serialize()

    @classmethod
    def from_database(cls, db_view: AnyView) -> Serializable:
        genotype = multineat.Genome()
        genotype.Deserialize(db_view.string)
        return cls(genotype)
