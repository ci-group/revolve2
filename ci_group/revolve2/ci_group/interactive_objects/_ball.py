from revolve2.simulation.scene import Color, MultiBodySystem, Pose, RigidBody
from revolve2.simulation.scene.geometry import GeometrySphere
from revolve2.simulation.scene.geometry.textures import Texture


class Ball(MultiBodySystem):
    """An intractable ball."""

    def __init__(
        self,
        radius: float,
        mass: float,
        pose: Pose = Pose(),
        is_static: bool = False,
        texture: Texture | None = None,
    ) -> None:
        """
        Initialize the ball as a plain ModularRobot.

        :param radius: The radius of the ball.
        :param mass: The mass of the ball.
        :param pose: The pose of the ball in the simulation.
        :param is_static: Whether the ball can be interacted with.
        :param texture: The texture of the ball.
        """
        super().__init__(pose=pose, is_static=is_static)

        if texture is None:
            texture = Texture(base_color=Color(255, 255, 255, 255))

        ball = GeometrySphere(pose=Pose(), mass=mass, radius=radius, texture=texture)
        rigid_body = RigidBody(
            initial_pose=Pose(),
            static_friction=1.0,
            dynamic_friction=1.0,
            geometries=[ball],
        )
        self.add_rigid_body(rigid_body)
