from ...body.base import ActiveHinge
from ._cpg_network_structure import CpgNetworkStructure, CpgPair


def active_hinges_to_cpg_network_structure_neighbor(
    active_hinges: list[ActiveHinge],
) -> tuple[CpgNetworkStructure, list[tuple[int, ActiveHinge]]]:
    """
    Create the structure of a cpg network based on a list of active hinges.

    The order of the active hinges matches the order of the cpgs.
    I.e. every active hinges has a corresponding cpg,
    and these are stored in the order the hinges are provided in.

    :param active_hinges: The active hinges to base the structure on.
    :returns: The created structure and a mapping between state indices and active hinges.
    """
    cpgs = CpgNetworkStructure.make_cpgs(len(active_hinges))
    connections: set[CpgPair] = set()

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

    cpg_network_structure = CpgNetworkStructure(cpgs, connections)

    return cpg_network_structure, [
        mapping for mapping in zip(cpg_network_structure.output_indices, active_hinges)
    ]
