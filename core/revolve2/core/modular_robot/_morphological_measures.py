from ._body import Body
from ._not_finalized_error import NotFinalizedError
from ._active_hinge import ActiveHinge
from ._brick import Brick
from ._module import Module
from typing import List


class MorphologicalMeasures:
    """
    Modular robot morphological measures.

    Some measures are based on the following paper:
    Miras, K., Haasdijk, E., Glette, K., Eiben, A.E. (2018).
    Search Space Analysis of Evolvable Robot Morphologies.\
    In: Sim, K., Kaufmann, P. (eds) Applications of Evolutionary Computation.
    EvoApplications 2018. Lecture Notes in Computer Science(), vol 10784. Springer, Cham.
    https://doi.org/10.1007/978-3-319-77538-8_47
    """

    bricks: List[Brick]
    active_hinges: List[ActiveHinge]

    """
    Vertical active hinges.

    This means their axis of rotation is vertical ('up/down').
    The hinge itself thus moves left and right.
    """
    active_hinges_vertical: List[ActiveHinge]

    """
    Horizontal active hinges.

    This means their axis of rotation is horizontal ('left/right').
    The hinge itself thus moves up and down.
    """
    active_hinges_horizontal: List[ActiveHinge]

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

    def __init__(self, body: Body) -> None:
        if not body.is_finalized:
            raise NotFinalizedError()

        self.__count_modules(body)

    def __count_modules(self, body: Body) -> None:
        pass

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
    def num_active_hinges_vertical(self) -> int:
        """
        Get the number of vertical active hinges.

        This means their axis of rotation is vertical ('up/down').
        The hinge itself thus moves left and right.

        :returns: The number of vertical active hinges.
        """
        return len(self.active_hinges_vertical)

    @property
    def num_active_hinges_horizontal(self) -> int:
        """
        Get the number of horizontal active hinges.

        This means their axis of rotation is horizontal ('left/right').
        The hinge itself thus moves up and down.

        :returns: The number of horizontal active hinges.
        """
        return len(self.active_hinges_horizontal)

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
        #   B B B B
        #   | | | |
        # C-B-B-B-B-B
        #   | | | |
        #   B B B B
        #
        # Active hinges are irrelevant because they can always be placed in between two modules without affecting this number.

        return self.num_bricks - ((self.num_bricks + 1) // 3)

    @property
    def single_neighbour_bricks_proportion(self) -> float:
        """
        Get the ratio between bricks with a single neighbour and with how many bricks that potentionally could have been if this set of modules was rearranged in an optimal way.

        :returns: The proportion.
        """
        if self.max_potential_single_neighbour_bricks == 0:
            return 0.0

        return (
            self.num_single_neighbour_bricks
            / self.max_potential_single_neighbour_bricks
        )
