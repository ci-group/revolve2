from revolve2.simulation.scene import MultiBodySystem, Pose, RigidBody

from ._terrain import Terrain


def convert_terrain(terrain: Terrain) -> MultiBodySystem:
    """
    Convert a terrain to a multi-body system.

    :param terrain: The terrain to convert.
    :returns: The created multi-body system.
    """
    multi_body_system = MultiBodySystem(pose=Pose(), is_static=True)
    rigid_body = RigidBody(
        initial_pose=Pose(),
        static_friction=1.0,
        dynamic_friction=1.0,
        geometries=[],
    )  # We use these friction values but in the future they should be set through the terrain description.
    multi_body_system.add_rigid_body(rigid_body)

    for geometry in terrain.static_geometry:
        rigid_body.geometries.append(geometry)
    return multi_body_system
