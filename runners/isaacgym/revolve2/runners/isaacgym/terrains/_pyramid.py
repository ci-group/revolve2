from isaacgym import gymapi
from isaacgym.terrain_utils import (
    SubTerrain,
    convert_heightfield_to_trimesh,
    pyramid_sloped_terrain,
)


def pyramid_terrain_generator(gym: gymapi.Gym, sim: gymapi.Sim) -> None:
    """
    Generate a pyramid terrain.

    :param gym: Isaac gym.
    :param sim: Isaac Gym sim.
    """
    plane_params = gymapi.PlaneParams()
    plane_params.normal = gymapi.Vec3(0, 0, 1)
    plane_params.distance = 5
    plane_params.static_friction = 1.0
    plane_params.dynamic_friction = 1.0
    plane_params.restitution = 0
    gym.add_ground(sim, plane_params)

    # TODO this is highly inefficient as it uses a heightmap, while it could be done with 8 vertices.

    w = 100
    l = 100

    horizontal_scale = 0.25  # [m]
    vertical_scale = 0.005  # [m]
    heightfield = pyramid_sloped_terrain(
        SubTerrain(
            width=w,
            length=l,
            vertical_scale=vertical_scale,
            horizontal_scale=horizontal_scale,
        ),
        slope=1.0,
        platform_size=0.0,
    ).height_field_raw
    vertices, triangles = convert_heightfield_to_trimesh(
        heightfield, horizontal_scale=0.5, vertical_scale=0.002
    )
    tm_params = gymapi.TriangleMeshParams()
    tm_params.nb_vertices = vertices.shape[0]
    tm_params.nb_triangles = triangles.shape[0]
    # TODO: offsets of the terrain, tweaked so the group of robots is centered for 100 population size
    tm_params.transform.p.x = -w / 4
    tm_params.transform.p.y = -l / 4
    tm_params.transform.p.z = -5
    gym.add_triangle_mesh(sim, vertices.flatten(), triangles.flatten(), tm_params)
