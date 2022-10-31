from isaacgym import gymapi
from isaacgym.terrain_utils import (
    SubTerrain,
    convert_heightfield_to_trimesh,
    wave_terrain,
)


def even_waves_terrain_generator(gym: gymapi.Gym, sim: gymapi.Sim) -> None:
    """
    Generate a terrain of waves.

    :param gym: Isaac gym.
    :param sim: Isaac Gym sim.
    """
    horizontal_scale = 0.0125  # [m] 0.25 0.0125
    vertical_scale = 0.00025  # [m] 0.005 0.00025
    heightfield = wave_terrain(
        SubTerrain(
            width=1000,
            length=1000,
            vertical_scale=vertical_scale,
            horizontal_scale=horizontal_scale,
        ),
        num_waves=50.0,
        amplitude=0.04,
    ).height_field_raw
    # heightfield = pyramid_stairs_terrain(SubTerrain(width=512, length=512, vertical_scale=vertical_scale, horizontal_scale=horizontal_scale), step_width=10., step_height=-0.3, platform_size=1.).height_field_raw
    vertices, triangles = convert_heightfield_to_trimesh(
        heightfield,
        horizontal_scale=horizontal_scale,
        vertical_scale=vertical_scale,
        slope_threshold=1.5,
    )
    tm_params = gymapi.TriangleMeshParams()
    tm_params.nb_vertices = vertices.shape[0]
    tm_params.nb_triangles = triangles.shape[0]
    # TODO: offsets of the terrain, tweaked to fit the centre perfectly with the robot
    tm_params.static_friction = 1.0
    tm_params.dynamic_friction = 1.0
    tm_params.transform.p.x = -3.25
    tm_params.transform.p.y = -3.25
    tm_params.transform.p.z = -0.125
    gym.add_triangle_mesh(sim, vertices.flatten(), triangles.flatten(), tm_params)
