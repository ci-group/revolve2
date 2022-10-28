from isaacgym import gymapi


def flat_terrain_generator(gym: gymapi.Gym, sim: gymapi.Sim) -> None:
    """
    Generate a flat terrain.

    :param gym: Isaac gym.
    :param sim: Isaac Gym sim.
    """
    plane_params = gymapi.PlaneParams()
    plane_params.normal = gymapi.Vec3(0, 0, 1)
    plane_params.distance = 0
    plane_params.static_friction = 1.0
    plane_params.dynamic_friction = 1.0
    plane_params.restitution = 0
    gym.add_ground(sim, plane_params)
