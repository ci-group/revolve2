from dataclasses import dataclass, field

from revolve2.modular_robot import ModularRobot
from revolve2.simulation.scene import MultiBodySystem, Pose, Scene, UUIDKey

from ._build_multi_body_systems import BodyToMultiBodySystemConverter
from ._convert_terrain import convert_terrain
from ._modular_robot_simulation_handler import ModularRobotSimulationHandler
from ._terrain import Terrain


@dataclass
class ModularRobotScene:
    """A scene of modular robots in a terrain."""

    terrain: Terrain
    """The terrain of the scene."""

    _robots: list[tuple[ModularRobot, Pose, bool]] = field(default_factory=list)
    """
    The robots in the scene.
    This is an owning collection; the robots are assigned ids when they are added, equal to their index in this list.
    """

    def add_robot(
        self, robot: ModularRobot, pose: Pose = Pose(), translate_z_aabb: bool = True
    ) -> None:
        """
        Add a robot to the scene.

        :param robot: The robot to add.
        :param pose: The pose of the robot.
        :param translate_z_aabb: Whether the robot should be translated upwards so it's T-pose axis-aligned bounding box is exactly on the ground. I.e. if the robot should be placed exactly on the ground. The pose parameters is still added afterwards.
        """
        # Add the robot to the robots list.
        self._robots.append(
            (
                robot,
                Pose(pose.position.copy(), pose.orientation.copy()),
                translate_z_aabb,
            )
        )

    def to_simulation_scene(
        self,
    ) -> tuple[Scene, dict[UUIDKey[ModularRobot], MultiBodySystem]]:
        """
        Convert this to a simulation scene.

        :returns: The created scene.
        """
        handler = ModularRobotSimulationHandler()
        scene = Scene(handler=handler)
        modular_robot_to_multi_body_system_mapping: dict[
            UUIDKey[ModularRobot], MultiBodySystem
        ] = {}

        # Add terrain
        scene.add_multi_body_system(convert_terrain(self.terrain))

        # Add robots
        converter = BodyToMultiBodySystemConverter()
        for robot, pose, translate_z_aabb in self._robots:
            # Convert all bodies to multi body systems and add them to the simulation scene
            (
                multi_body_system,
                body_to_multi_body_system_mapping,
            ) = converter.convert_robot_body(
                body=robot.body, pose=pose, translate_z_aabb=translate_z_aabb
            )
            scene.add_multi_body_system(multi_body_system)
            handler.add_robot(
                robot.brain.make_instance(), body_to_multi_body_system_mapping
            )
            modular_robot_to_multi_body_system_mapping[
                UUIDKey(robot)
            ] = multi_body_system

        return scene, modular_robot_to_multi_body_system_mapping
