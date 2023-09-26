from revolve2.actor_controllers.cpg import CpgNetworkStructure, CpgPair
from revolve2.simulation.actor import Actor

from .. import Body
from .._common import ActiveHinge


def body_to_actor_and_cpg_network_structure_neighbour(
    body: Body,
) -> tuple[Actor, CpgNetworkStructure]:
    """
    Convert a body to an actor and get it's corresponding cpg network structure.

    :param body: The body to convert.
    :returns: A tuple of the actor and cpg network structure.
    """
    actor, dof_ids = body.to_actor()
    id_to_hinge = {
        active_hinge.id: active_hinge for active_hinge in body.find_active_hinges()
    }
    active_hinges = [id_to_hinge[dof_id] for dof_id in dof_ids]
    cpg_network_structure = active_hinges_to_cpg_network_structure_neighbor(
        active_hinges
    )

    return actor, cpg_network_structure


def active_hinges_to_cpg_network_structure_neighbor(
    active_hinges: list[ActiveHinge],
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

    return CpgNetworkStructure(cpgs, connections)
