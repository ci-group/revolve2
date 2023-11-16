import numpy as np
import random
import math
from queue import Queue
from typing import Any, Dict, Tuple, Set, List, Self
from dataclasses import dataclass

from revolve2.modular_robot import (
    ActiveHinge,
    Body,
    Brick,
    Core,
    Module,
    MorphologicalMeasures,
)
from revolve2.modular_robot import Directions
from ..protocols import GenotypeInitParams, IGenotype


@dataclass
class _Module:
    position: Tuple[int, int, int]
    forward: Tuple[int, int, int]
    up: Tuple[int, int, int]
    chain_length: int
    module_reference: Module


class _CAGenotype:
    def __init__(self, init_state=[], iterations=1, rule_set={}) -> None:
        self.core_position = [0, 0]

        self.ca_grid = []

        self.init_state = init_state
        self.iterations = iterations
        self.rule_set = rule_set

    def copy(self, init_state):
        return self.__class__(init_state, self.iterations, self.rule_set.copy())

    def generate_body(self):
        domain = self.init_state
        for _ in range(self.iterations):
            domain = self.update_grid(domain)

        self.ca_grid = domain.copy()

    def update_grid(self, grid):
        rows, cols = len(grid), len(grid[0])
        domain_next = grid.copy()
        for i in range(1, rows - 1):
            for j in range(1, cols - 1):
                neighborhood = (
                    grid[i + 1][j],
                    grid[i - 1][j],
                    grid[i][j + 1],
                    grid[i][j - 1],
                )

                if self.rule_set.get(neighborhood) is not None:
                    cell_value = self.rule_set.get(neighborhood)
                    domain_next[i][j] = cell_value

        return domain_next

    def get_full_grid(self):
        return self.ca_grid

    def get_separate_grids(self):
        rows, cols = len(self.ca_grid), len(self.ca_grid[0])
        module_grid = np.zeros((rows, cols))
        orientation_grid = np.zeros((rows, cols))
        for i in range(rows):
            for j in range(cols):
                module_grid[i][j] = self.ca_grid[i][j]
                orientation_grid[i][j] = self.ca_grid[i][j]

        return module_grid, orientation_grid

    # Temporary function for testing
    def set_core(self, i, j):
        self.core_position = [i, j]
        self.ca_grid[i][j] = 3

    def mutate(self):
        possible_values = [0.0, 1.0, 2.0]
        new_key = tuple(random.choice(possible_values) for _ in range(4))
        new_value = random.choice(possible_values[1:])

        # Choose an existing key to replace
        existing_key = random.choice(list(self.rule_set.keys()))
        self.rule_set[new_key] = new_value
        del self.rule_set[existing_key]

    def generate_random_genotype(self, n):
        possible_values = [0.0, 1.0, 2.0]
        gene = {}

        for _ in range(n):
            # Generate n random key value pairs
            new_key = tuple(random.choice(possible_values) for _ in range(4))
            new_value = random.choice(possible_values[1:])
            gene[new_key] = new_value

        self.rule_set = gene

    def develop(
        self,
    ) -> Body:
        """
        Maps CA genotype grid to revolve2 robot (NOT FINISHED)

        :param genotype: object of CA genotype representation
        :returns: revolve2 robot body
        :raises RuntimeError: In case a module is encountered that is not supported.
        """
        max_parts = 100

        to_explore: Queue[_Module] = Queue()
        grid: Set[Tuple[int, int, int]] = set()

        body = Body()
        ca_grid = self.ca_grid
        core_pos = self.core_position

        to_explore.put(
            _Module((core_pos[0], core_pos[1], 0), (0, -1, 0), (0, 0, 1), 0, body.core)
        )
        grid.add((0, 0, 0))
        part_count = 1

        while not to_explore.empty():
            module = to_explore.get()

            children: List[Tuple[Directions, int]] = []  # child index, rotation

            if isinstance(module.module_reference, Core):
                children.append((Directions.FRONT, 0))
                children.append((Directions.LEFT, 1))
                children.append((Directions.BACK, 2))
                children.append((Directions.RIGHT, 3))
            elif isinstance(module.module_reference, Brick):
                children.append((Directions.FRONT, 0))
                children.append((Directions.LEFT, 1))
                children.append((Directions.RIGHT, 3))
            elif isinstance(module.module_reference, ActiveHinge):
                children.append((Directions.FRONT, 0))
            else:  # Should actually never arrive here but just checking module type to be sure
                raise RuntimeError()

            for direction, rotation in children:
                if part_count < max_parts:
                    child = _add_child(ca_grid, module, direction, rotation, grid)
                    if child is not None:
                        to_explore.put(child)
                        part_count += 1

        body.finalize()

        return body


def _module_from_ca(
    ca_grid,
    position: Tuple[int, int, int],
) -> Tuple[Any, int]:
    """
    Identifies which module type is present in ca_grid[position]

    :param position: Position of the module.
    :param chain_length: Tree distance of the module from the core.
    :param ca_grid: cellular automata grid that must define body
    :returns: (module type, rotation)
    """

    # print("current position on ca_grid: ", position)
    # print("ca_grid value at position: ", ca_grid[position[0]][position[1]])

    if ca_grid[position[0]][position[1]] == 1:
        module_type = Brick
    elif ca_grid[position[0]][position[1]] == 2:
        module_type = ActiveHinge
    elif ca_grid[position[0]][position[1]] == 3:
        module_type = Core
    else:
        module_type = None

    rotation = 0  # ca_grid[position[0]][position[1]]

    # print("module_type: ", module_type)

    return (module_type, rotation)


def _add_child(
    ca_grid,
    module: _Module,
    direction: Directions,
    rotation: int,
    grid: Set[Tuple[int, int, int]],
) -> _Module | None:
    forward = _rotate(module.forward, module.up, rotation)
    position = _add(module.position, forward)
    chain_length = module.chain_length + 1

    # if grid cell is occupied, don't make a child
    # else, set cell as occupied
    if position in grid:
        return None
    else:
        grid.add(position)

    child_type, orientation = _module_from_ca(ca_grid, position)
    if child_type is None:
        return None
    up = _rotate(module.up, forward, orientation)

    child = child_type(orientation * (math.pi / 2.0))
    module.module_reference.set_child(child, direction)

    return _Module(
        position,
        forward,
        up,
        chain_length,
        child,
    )


# Use pyrr functions instead
def _add(a: Tuple[int, int, int], b: Tuple[int, int, int]) -> Tuple[int, int, int]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _timesscalar(a: Tuple[int, int, int], scalar: int) -> Tuple[int, int, int]:
    return (a[0] * scalar, a[1] * scalar, a[2] * scalar)


def _cross(a: Tuple[int, int, int], b: Tuple[int, int, int]) -> Tuple[int, int, int]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _dot(a: Tuple[int, int, int], b: Tuple[int, int, int]) -> int:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _rotate(
    a: Tuple[int, int, int], b: Tuple[int, int, int], angle: int
) -> Tuple[int, int, int]:
    """
    Rotates vector a a given angle around b.

    Angle from [0,1,2,3].
    90 degrees each.

    :param a: Vector a.
    :param b: Vector b.
    :param angle: The angle to rotate.
    :returns: A copy of a, rotated.
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

    return _add(
        _add(
            _timesscalar(a, cosangle),
            _timesscalar(_cross(b, a), sinangle),
        ),
        _timesscalar(b, _dot(b, a) * (1 - cosangle)),
    )


@dataclass
class CAInitParameters(GenotypeInitParams):
    domain_size: int
    iterations: int
    nr_rules: int


class CAGenotype(IGenotype):
    _ca_type: _CAGenotype
    params: CAInitParameters
    rule_set: Dict[Tuple[int, int, int, int], int]

    def __init__(self, params: CAInitParameters) -> None:
        domain = np.zeros((params.domain_size, params.domain_size))
        self._ca_type = _CAGenotype(
            init_state=domain, iterations=params.iterations, rule_set={}
        )
        self.params = params

    def develop(self) -> Body:
        self._ca_type.generate_body()
        self._ca_type.set_core(
            self.params.domain_size // 2, self.params.domain_size // 2
        )
        return self._ca_type.develop()

    def copy(self) -> Self:
        newitem = self.__class__(self.params)
        newitem._ca_type.rule_set = self._ca_type.rule_set.copy()
        return newitem

    def mutate(self) -> Self:
        newitem = self.copy()
        newitem._ca_type.mutate()
        return newitem

    @classmethod
    def crossover(cls, _rhs: Self, _lhs: Self, /) -> Self:
        raise NotImplementedError

    @classmethod
    def random(cls, params: CAInitParameters, rng: np.random.Generator) -> Self:
        newitem = cls(params)
        newitem._ca_type.generate_random_genotype(params.nr_rules)
        return newitem
