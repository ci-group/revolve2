from typing import List, Set

from revolve2.actor_controllers.cpg import CpgNetworkStructure, CpgPair
from revolve2.core.modular_robot import ActiveHinge


def make_cpg_network_structure_neighbour(
    active_hinges: List[ActiveHinge],
) -> CpgNetworkStructure:
    """
    Create the structure of a cpg network based on a list of active hinges.

    The order of the active hinges matches the order of the cpgs.
    I.e. every active hinges has a corresponding cpg,
    and these are stored in the order the hinges are provided in.

    :param active_hinges: The active hinges to base the structure on.
    :returns: The created structure.
    """
    cpgs = CpgNetworkStructure.make_cpgs(len(active_hinges))
    connections: Set[CpgPair] = set()

    active_hinge_to_cpg = {
        active_hinge: cpg for active_hinge, cpg in zip(active_hinges, cpgs)
    }

    for active_hinge, cpg in zip(active_hinges, cpgs):
        neighbours = [
            n
            for n in active_hinge.neighbours(within_range=2)
            if isinstance(n, ActiveHinge)
        ]
        connections = connections.union(
            [CpgPair(cpg, active_hinge_to_cpg[neighbour]) for neighbour in neighbours]
        )

    return CpgNetworkStructure(cpgs, connections)
