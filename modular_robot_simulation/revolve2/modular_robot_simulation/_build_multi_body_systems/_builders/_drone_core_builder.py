from revolve2.modular_robot.body.base import DroneCore
from revolve2.simulation.scene import AABB, MultiBodySystem, Pose, RigidBody
from revolve2.simulation.scene.geometry import GeometryBox
from revolve2.simulation.scene.geometry.textures import Texture

from .._body_to_multi_body_system_mapping import BodyToMultiBodySystemMapping
from .._convert_color import convert_color
from .._unbuilt_child import UnbuiltChild
from ._builder import Builder


class DroneCoreBuilder(Builder):
    """A Builder for Cores."""

    _module: DroneCore

    def __init__(self, module: DroneCore, rigid_body: RigidBody, pose: Pose):
        """
        Initialize the Core Builder.

        :param module: The module to be built.
        :param rigid_body: The rigid body for the module to be built on.
        :param slot_pose: The slot pose of the module.
        """
        self._module = module
        self._rigid_body = rigid_body
        self._pose = pose

    def build(
        self,
        multi_body_system: MultiBodySystem,
        body_to_multi_body_system_mapping: BodyToMultiBodySystemMapping,
    ) -> list[UnbuiltChild]:
        """
        Build a module onto the Robot.

        :param multi_body_system: The multi body system of the robot.
        :param body_to_multi_body_system_mapping: A mapping from body to multi-body system
        :return: The next children to be built.
        """
        self._rigid_body.geometries.append(
            GeometryBox(
                pose=self._pose,
                mass=self._module.mass,
                texture=Texture(base_color=convert_color(self._module.color)),
                aabb=AABB(self._module.bounding_box),
            )
        )

        tasks = []
        for sensor in self._module.sensors.get_all_sensors():
            unbuilt = UnbuiltChild(child_object=sensor, rigid_body=self._rigid_body)
            unbuilt.make_pose(position=self._pose.position)
            tasks.append(unbuilt)

        for child in self._module._children.values():
            unbuilt = UnbuiltChild(
                child_object=child,
                rigid_body=self._rigid_body,
            )
            unbuilt.make_pose(
                position=self._pose.position,
                orientation=self._pose.orientation
            )
            tasks.append(unbuilt)

        return tasks
