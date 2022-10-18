from ._body import Body
from ._not_finalized_error import NotFinalizedError
from ._active_hinge import ActiveHinge
from ._brick import Brick
from ._module import Module
from ._core import Core
from typing import List, Optional, Tuple
import numpy as np
import math
from pyrr import Vector3, Quaternion


class MorphologicalMeasures:
    """
    Modular robot morphological measures.

    Only works for robot with only right angle module rotations (90 degrees).
    Some measures only work for 2d robots, which is noted in their docstring.

    Some measures are based on the following paper:
    Miras, K., Haasdijk, E., Glette, K., Eiben, A.E. (2018).
    Search Space Analysis of Evolvable Robot Morphologies.
    In: Sim, K., Kaufmann, P. (eds) Applications of Evolutionary Computation.
    EvoApplications 2018. Lecture Notes in Computer Science(), vol 10784. Springer, Cham.
    https://doi.org/10.1007/978-3-319-77538-8_47
    """

    body_as_grid: List[List[List[Optional[Module]]]]
    core_grid_position: Tuple[int, int, int]

    is_2d: bool

    core: Core
    bricks: List[Brick]
    active_hinges: List[ActiveHinge]

    """If all slots of the core are filled with other modules."""
    core_is_filled: bool

    """Bricks which have all slots filled with other modules."""
    filled_bricks: List[Brick]

    """Active hinges which have all slots filled with other modules."""
    filled_active_hinges: List[ActiveHinge]

    """
    Bricks that are only connected to one other module.

    Both children and parent are counted.
    """
    single_neighbour_bricks: List[Brick]

    """
    Bricks that are connected to exactly two other modules.

    Both children and parent are counted.
    """
    double_neighbour_bricks: List[Brick]

    """
    Active hinges that are connected to exactly two other modules.

    Both children and parent are counted.
    """
    double_neighbour_active_hinges: List[ActiveHinge]

    """
    X-axis symmetry according to the paper.

    X-axis is defined as forward/backward for the core module
    """
    x_symmetry: float

    """
    Y-axis symmetry according to the paper.

    Y-axis is defined as left/right for the core module.
    """
    y_symmetry: float

    """
    Z-axis symmetry according to the paper.

    Z-axis is defined as up/down for the core module.
    """
    z_symmetry: float

    def __init__(self, body: Body) -> None:
        if not body.is_finalized:
            raise NotFinalizedError()

        self.body_as_grid, self.core_grid_position = body.to_grid()

        self.is_2d = self.__calculate_is_2d(body)
        self.__aggregate_modules(body)
        self.core_is_filled = self.__calculate_core_is_filled()
        self.filled_bricks = self.__calculate_filled_bricks()
        self.filled_active_hinges = self.__calculate_filled_active_hinges()
        self.single_neighbour_bricks = self.__calculate_single_neighbour_bricks()
        self.double_neighbour_bricks = self.__calculate_double_neighbour_bricks()
        self.double_neighbour_active_hinges = (
            self.__calculate_double_neighbour_active_hinges()
        )
        self.__calculate_symmetries()

    @classmethod
    def __calculate_is_2d(cls, body: Body) -> bool:
        return cls.__calculate_is_2d_recur(body.core)

    @classmethod
    def __calculate_is_2d_recur(cls, module: Module) -> bool:
        return all(
            [np.isclose(module.rotation, 0.0)]
            + [
                cls.__calculate_is_2d_recur(child)
                for child in module.children
                if child is not None
            ]
        )

    def __aggregate_modules(self, body: Body) -> None:
        self.__aggregate_modules_recur(body.core)

    def __aggregate_modules_recur(self, module: Module) -> None:
        if isinstance(module, Core):
            self.core = module
        elif isinstance(module, Brick):
            self.bricks.append(module)
        elif isinstance(module, ActiveHinge):
            self.active_hinges.append(module)
        else:
            raise NotImplementedError()

        for child in module.children:
            self.__aggregate_modules_recur(child)

    def __calculate_core_is_filled(self) -> bool:
        return all([child is not None for child in self.core.children])

    def __calculate_filled_bricks(self) -> List[Brick]:
        return [
            brick
            for brick in self.bricks
            if all([child is not None for child in brick.children])
        ]

    def __calculate_filled_active_hinges(self) -> List[ActiveHinge]:
        return [
            active_hinge
            for active_hinge in self.active_hinges
            if all([child is not None for child in active_hinge.children])
        ]

    def __calculate_single_neighbour_bricks(self) -> List[Brick]:
        return [
            brick
            for brick in self.bricks
            if all([child is None for child in brick.children])
        ]

    def __calculate_double_neighbour_bricks(self) -> List[Brick]:
        return [
            brick
            for brick in self.bricks
            if sum([0 if child is None else 1 for child in brick.children]) == 1
        ]

    def __calculate_double_neighbour_active_hinges(self) -> List[Brick]:
        return [
            active_hinge
            for active_hinge in self.active_hinges
            if sum([0 if child is None else 1 for child in active_hinge.children]) == 1
        ]

    def __calculate_symmetries(self) -> None:
        raise NotImplementedError()

    @property
    def bounding_box_depth(self) -> int:
        """
        Get the depth of the bounding box around the body.

        Forward/backward axis for the core module.

        :returns: The depth.
        """
        return len(self.body_as_grid)

    @property
    def bounding_box_width(self) -> int:
        """
        Get the width of the bounding box around the body.

        Right/left axis for the core module.

        :returns: The width.
        """
        return len(self.body_as_grid[0])

    @property
    def bounding_box_height(self) -> int:
        """
        Get the height of the bounding box around the body.

        Up/down axis for the core module.

        :returns: The height.
        """
        return len(self.body_as_grid[0][0])

    @property
    def num_modules(self) -> int:
        """
        Get the number of modules.

        :returns: The number of modules.
        """
        return 1 + self.num_bricks + self.num_active_hinges

    @property
    def num_bricks(self) -> int:
        """
        Get the number of bricks.

        :returns: The number of bricks.
        """
        return len(self.bricks)

    @property
    def num_active_hinges(self) -> int:
        """
        Get the number of active hinges.

        :returns: The number of active hinges.
        """
        return len(self.active_hinges)

    @property
    def num_filled_bricks(self) -> int:
        """
        Get the number of bricks which have all slots filled with other modules.

        :returns: The number of bricks.
        """
        return len(self.filled_bricks)

    @property
    def num_filled_active_hinges(self) -> int:
        """
        Get the number of bricks which have all slots filled with other modules.

        :returns: The number of bricks.
        """
        return len(self.filled_active_hinges)

    @property
    def num_filled_modules(self) -> int:
        """
        Get the number of modules which have all slots filled with other modules, including the core.

        :returns: The number of modules.
        """
        return (
            len(self.num_filled_bricks)
            + len(self.num_active_hinges)
            + (1 if self.core_is_filled else 0)
        )

    @property
    def max_potentionally_filled_core_and_bricks(self) -> int:
        """
        Get the maximum number of core and bricks that could potentially be filled with this set of modules if rearranged in an optimal way.

        This calculates 'b_max' from the paper.

        :returns: The calculated number.
        """
        # Snake-like is an optimal arrangement.
        #
        #   H H H H
        #   | | | |
        # H-C-B-B-B-H
        #   | | | |
        #   H H H H
        #
        # Every extra brick(B) requires 3 modules:
        # The bricks itself and two other modules for its sides(here displayed as H).
        # However, the core and final brick require three each to fill, which is cheaper than another brick.
        #
        # Expected sequence:
        # | num modules | 1 2 3 4 5 6 7 8 9 10 11 12 14
        # | return val  | 0 0 0 0 1 1 1 2 2 2  3  3  3

        pot_max_filled = max(0, (self.num_modules - 2) // 3)

        # Enough bricks must be available for this strategy.
        # We can count the core as the first brick.
        pot_max_filled = min(pot_max_filled, 1 + self.num_bricks)

        return pot_max_filled

    @property
    def filled_core_and_bricks_proportion(self) -> float:
        """
        Get the ratio between filled cores and bricks and how many that potentially could have been if this set of modules was rearranged in an optimal way.

        This calculates 'branching' from the paper.

        :returns: The proportion.
        """
        if self.max_potentionally_filled_core_and_bricks == 0:
            return 0.0

        return (
            self.filled_bricks + (1 if self.core_is_filled else 0)
        ) / self.max_potentionally_filled_core_and_bricks

    @property
    def num_single_neighbour_bricks(self) -> int:
        """
        Get the number of bricks that are only connected to one other module.

        Both children and parent are counted.

        :returns: The number of bricks.
        """
        return len(self.single_neighbour_bricks)

    @property
    def max_potential_single_neighbour_bricks(self) -> int:
        """
        Get the maximum number of bricks that could potentially have only one neighbour if this set of modules was rearranged in an optimal way.

        This calculates "l_max" from the paper.

        :returns: The calculated number.
        """
        # Snake-like is an optimal arrangement.
        #
        #   B B B B B
        #   | | | | |
        # B-C-B-B-B-B-B
        #   | | | | |
        #   B B B B B
        #
        # Active hinges are irrelevant because they can always be placed in between two modules without affecting this number.
        #
        # Expected sequence:
        # | num bricks | 0 1 2 3 4 5 6 7 8 9
        # | return val | 0 1 2 3 4 4 5 6 6 7

        return self.num_bricks - max(0, (self.num_bricks - 2) // 3)

    @property
    def single_neighbour_brick_proportion(self) -> float:
        """
        Get the ratio between bricks with a single neighbour and with how many bricks that potentionally could have been if this set of modules was rearranged in an optimal way.

        This calculates limb proportion(extremities) from the paper.

        :returns: The proportion.
        """
        if self.max_potential_single_neighbour_bricks == 0:
            return 0.0

        return (
            self.num_single_neighbour_bricks
            / self.max_potential_single_neighbour_bricks
        )

    @property
    def num_double_neighbour_bricks(self) -> int:
        """
        Get the number of bricks that are connected to exactly two other modules.

        Both children and parent are counted.

        :returns: The number of bricks.
        """
        return len(self.double_neighbour_bricks)

    @property
    def num_double_neighbour_active_hinges(self) -> int:
        """
        Get the number of active hinges that are connected to exactly two other modules.

        Both children and parent are counted.

        :returns: The number of active hinges.
        """
        return len(self.double_neighbour_active_hinges)

    @property
    def potential_double_neighbour_bricks_and_active_hinges(self) -> int:
        """
        Get the maximum number of bricks and active hinges that could potentially have exactly two neighbours if this set of modules was rearranged in an optimal way.

        This calculates e_max from the paper.

        :returns: The calculated number.
        """
        #
        # C-M-M-M-M-M
        #
        # Snake in direction is optimal, no matter whether modules are bricks or active hinges.
        #
        # Simply add up the number of bricks and active hinges and subtract 1 for the final module.

        return max(0, self.num_bricks + self.num_active_hinges - 1)

    @property
    def double_neighbour_brick_and_active_hinge_proportion(self) -> float:
        """
        Get the ratio between the number of bricks and active hinges with exactly two neighbours and how many that could potentially have been if this set of modules was rearranged in an optimal way.

        This calculate length of limbs proportion(extensiveness) from the paper.

        :returns: The proportion.
        """
        if self.potential_double_neighbour_bricks_and_active_hinges == 0:
            return 0.0

        return (
            self.num_double_neighbour_bricks + self.num_double_neighbour_active_hinges
        ) / self.potential_double_neighbour_bricks_and_active_hinges

    @property
    def bounding_box_volume(self) -> int:
        """
        Get the volume of the bounding box.

        This calculates m_area from the paper.

        :returns: The volume.
        """
        return (
            self.bounding_box_width * self.bounding_box_height * self.bounding_box_depth
        )

    @property
    def bounding_box_volume_coverage(self) -> float:
        """
        The proportion of the bounding box that is filled with modules.

        This calculates 'coverage' from the paper.

        :returns: The proportion.
        """
        return self.num_modules / self.bounding_box_volume

    @property
    def branching(self) -> float:
        """
        Get the 'branching' measurement from the paper.

        Alias for filled_core_and_bricks_proportion.

        :returns: Branching measurement.
        """
        return self.filled_core_and_bricks_proportion

    @property
    def limbs(self) -> float:
        """
        Get the 'limbs' measurement from the paper.

        Alias for single_neighbour_brick_proportion.

        :returns: Limbs measurement.
        """
        return self.single_neighbour_brick_proportion

    @property
    def length_of_limbs(self) -> float:
        """
        Get the 'length of limbs' measurement from the paper.

        Alias for double_neighbour_brick_and_active_hinge_proportion.

        :returns: Length of limbs measurement.
        """
        return self.double_neighbour_brick_and_active_hinge_proportion

    @property
    def coverage(self) -> float:
        """
        Get the 'coverage' measurement from the paper.

        Alias for bounding_box_volume_coverage.

        :returns: Coverage measurement.
        """
        return self.bounding_box_volume_coverage

    @property
    def proportion_2d(self) -> float:
        """
        Get the 'proportion' measurement from the paper.

        Only for 2d robots.

        :returns: Proportion measurement.
        """
        assert self.is_2d

        return min(self.bounding_box_depth, self.bounding_box_width) / max(
            self.bounding_box_depth, self.bounding_box_width
        )

    @property
    def symmetry(self) -> float:
        """
        Get the 'symmetry' measurement from the paper, but extended to 3d.

        :returns: Symmetry measurement.
        """
        return max(self.x_symmetry, self.y_symmetry, self.z_symmetry)
