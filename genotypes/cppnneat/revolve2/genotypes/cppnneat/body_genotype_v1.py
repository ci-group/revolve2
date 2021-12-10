import math
from dataclasses import dataclass
from queue import Queue
from typing import Any, List, Optional, Set, Tuple

import multineat
from revolve2.core.database import Data
from revolve2.core.database.serialize import Serializable, SerializeError
from revolve2.core.modular_robot import ActiveHinge
from revolve2.core.modular_robot import Body
from revolve2.core.modular_robot import Body as ModularRobotBody
from revolve2.core.modular_robot import Brick, Core, Module
from revolve2.core.optimization.ea.modular_robot import BodyGenotype

from .bodybrain_base import BodybrainBase


class BodyGenotypeV1(BodyGenotype, BodybrainBase["BodyGenotypeV1"], Serializable):
    @classmethod
    def random(
        cls,
        innov_db: multineat.InnovationDatabase,
        rng: multineat.RNG,
        multineat_params: multineat.Parameters,
        output_activation_func: multineat.ActivationFunction,
        num_initial_mutations: int,
    ) -> BodybrainBase:
        return super(BodyGenotypeV1, cls).random(
            innov_db,
            rng,
            multineat_params,
            output_activation_func,
            5,  # bias(always 1), pos_x, pos_y, pos_z, chain_length
            5,  # empty, brick, activehinge, rot0, rot90
            num_initial_mutations,
        )

    @dataclass
    class _Module:
        position: Tuple[int, int, int]
        forward: Tuple[int, int, int]
        up: Tuple[int, int, int]
        chain_length: int
        module_reference: Module

    def develop(self) -> ModularRobotBody:
        max_parts = 10

        body_net = multineat.NeuralNetwork()
        self._genotype.BuildPhenotype(body_net)

        to_explore: Queue[self.Module] = Queue()
        grid: Set[Tuple(int, int, int)] = set()

        body = Body()

        to_explore.put(self._Module((0, 0, 0), (0, -1, 0), (0, 0, 1), 0, body.core))
        grid.add((0, 0, 0))
        part_count = 1

        while not to_explore.empty():
            module: self._Module = to_explore.get()

            children: List[Tuple[int, int]] = []  # child index, rotation

            if module.module_reference.type == Module.Type.CORE:
                children.append((Core.FRONT, 0))
                children.append((Core.LEFT, 1))
                children.append((Core.BACK, 2))
                children.append((Core.RIGHT, 3))
            elif module.module_reference.type == Module.Type.BRICK:
                children.append((Brick.FRONT, 0))
                children.append((Brick.LEFT, 1))
                children.append((Brick.RIGHT, 3))
            elif module.module_reference.type == Module.Type.ACTIVE_HINGE:
                children.append((ActiveHinge.ATTACHMENT_INDEX, 0))
            else:  # Should actually never arrive here but just checking module type to be sure
                raise RuntimeError()

            for (index, rotation) in children:
                if part_count < max_parts:
                    child = self._add_child(body_net, module, index, rotation, grid)
                    if child is not None:
                        to_explore.put(child)
                        part_count += 1

        return body

    @staticmethod
    def _evaluate_cppg(
        body_net: multineat.NeuralNetwork,
        position: Tuple[int, int, int],
        chain_length: int,
    ) -> Tuple[Any, int]:
        """
        get module type, orientation
        """
        body_net.Input(
            [1.0, position[0], position[1], position[2], chain_length]
        )  # 1.0 is the bias input
        body_net.Activate()
        outputs = body_net.Output()

        # get module type from output probabilities
        type_probs = [outputs[0], outputs[1], outputs[2]]
        types = [None, Brick, ActiveHinge]
        module_type = types[type_probs.index(min(type_probs))]

        # get rotation from output probabilities
        rotation_probs = [outputs[3], outputs[4]]
        rotation = rotation_probs.index(min(rotation_probs))

        return (module_type, rotation)

    @classmethod
    def _add_child(
        cls,
        body_net: multineat.NeuralNetwork,
        module: _Module,
        child_index: int,
        rotation: int,
        grid: Set[Tuple[int, int, int]],
    ) -> Optional[_Module]:
        forward = cls._rotate(module.forward, module.up, rotation)
        position = cls._add(module.position, forward)
        chain_length = module.chain_length + 1

        # if grid cell is occupied, don't make a child
        # else, set cell as occupied
        if position in grid:
            return None
        else:
            grid.add(position)

        child_type, orientation = cls._evaluate_cppg(body_net, position, chain_length)
        if child_type is None:
            return None
        up = cls._rotate(module.up, forward, orientation)

        child = child_type(orientation * (math.pi / 2.0))
        module.module_reference.children[child_index] = child

        return cls._Module(
            position,
            forward,
            up,
            chain_length,
            child,
        )

    @classmethod
    def _add(
        cls, a: Tuple[int, int, int], b: Tuple[int, int, int]
    ) -> Tuple[int, int, int]:
        return tuple(map(sum, zip(a, b)))

    @classmethod
    def _timesscalar(cls, a: Tuple[int, int, int], scalar: int) -> Tuple[int, int, int]:
        return (a[0] * scalar, a[1] * scalar, a[2] * scalar)

    @classmethod
    def _cross(
        cls, a: Tuple[int, int, int], b: Tuple[int, int, int]
    ) -> Tuple[int, int, int]:
        return (
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        )

    @classmethod
    def _dot(cls, a: Tuple[int, int, int], b: Tuple[int, int, int]) -> int:
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    @classmethod
    def _rotate(
        cls, a: Tuple[int, int, int], b: Tuple[int, int, int], angle: int
    ) -> Tuple[int, int, int]:
        """
        rotates a around b. angle from [0,1,2,3]. 90 degrees each
        """
        cosangle: int
        sinangle: int
        if angle == 0:
            cosangle = 1
            sinangle = 0
        elif angle == 1:
            cosangle = 0
            sinangle = 1
        elif angle == 2:
            cosangle = -1
            sinangle = 0
        else:
            cosangle = 0
            sinangle = -1

        return cls._add(
            cls._add(
                cls._timesscalar(a, cosangle),
                cls._timesscalar(cls._cross(b, a), sinangle),
            ),
            cls._timesscalar(b, cls._dot(b, a) * (1 - cosangle)),
        )

    def serialize(self) -> Data:
        return self._genotype.Serialize()

    @classmethod
    def deserialize(cls, data: Data) -> Serializable:
        if type(data) != str:
            raise SerializeError()
        genotype = multineat.Genome()
        genotype.Deserialize(data)
        return cls(genotype)
