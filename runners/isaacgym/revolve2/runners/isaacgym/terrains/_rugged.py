from isaacgym import gymapi
from isaacgym.terrain_utils import (
    SubTerrain,
    convert_heightfield_to_trimesh,
    random_uniform_terrain,
)


def rugged_terrain_generator(gym: gymapi.Gym, sim: gymapi.Sim) -> None:
    """
    Generate a rugged terrain.

    :param gym: Isaac gym.
    :param sim: Isaac Gym sim.
    """
    horizontal_scale = 0.25  # [m]
    vertical_scale = 0.005  # [m]
    heightfield = random_uniform_terrain(
        SubTerrain(
            width=512,
            length=512,
            vertical_scale=vertical_scale,
            horizontal_scale=horizontal_scale,
        ),
        min_height=-0.9,
        max_height=0.9,
        step=0.5,
        downsampled_scale=0.9,
    ).height_field_raw
    vertices, triangles = convert_heightfield_to_trimesh(
        heightfield,
        horizontal_scale=0.0125,
        vertical_scale=0.00025,
        slope_threshold=1.5,
    )
    tm_params = gymapi.TriangleMeshParams()
    tm_params.nb_vertices = vertices.shape[0]
    tm_params.nb_triangles = triangles.shape[0]
    # TODO: offsets of the terrain, tweaked to fit the centre perfectly with the robot
    tm_params.transform.p.x = -3.25
    tm_params.transform.p.y = -3.25
    tm_params.transform.p.z = -0.7  # for pyramid it was tm_params.transform.p.z = 0.274
    gym.add_triangle_mesh(sim, vertices.flatten(), triangles.flatten(), tm_params)
