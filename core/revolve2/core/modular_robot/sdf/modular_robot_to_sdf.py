from __future__ import annotations

import math
import xml.dom.minidom as minidom
import xml.etree.ElementTree as xml
from dataclasses import dataclass
from typing import List, Tuple, Union, cast

import numpy as np
import regex
import scipy.spatial.transform
from pyrr import Quaternion, Vector3
from revolve2.core.modular_robot import (ActiveHinge, Brick, Core,
                                         ModularRobot, Module)


def modular_robot_to_sdf(
    robot: ModularRobot,
    model_name: str,
    position: Vector3,
    orientation: Quaternion,
) -> str:
    """
    Create the sdf representation of a modular robot.

    :param robot: The robot
    :param model_name: Model_name to use in the sdf.
        Must consist only of letters, numbers, and underscores.
        Must start with a letter.
    :param position: Position of the robot.
    :param orientation: Orientation of the robot.
        Euler angles. XYZ. Extrinsic.
    :raises NotImplementedError: If uninmplemented module is encountered.
    """
    return _ModularRobotToSdf().modular_robot_to_sdf(
        robot, model_name, position, orientation
    )


# TODO currently only 90 degree angles are supported
#      upgrade to any degree angles
#      in the process, upgrade the rotation matrix implementation
#      to quaternions
#      a quick attempt was already made but for some reason
#      there were precious errors, so take care.
class _Rotation:
    _matrix: np.ndarray

    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        xint = self._float_rotation_to_discrete(x)
        yint = self._float_rotation_to_discrete(y)
        zint = self._float_rotation_to_discrete(z)

        self._matrix = np.ndarray(shape=(3, 3), dtype=int)
        self._matrix[0][0] = self._cos(zint) * self._cos(yint)
        self._matrix[0][1] = self._cos(zint) * self._sin(yint) * self._sin(
            xint
        ) - self._sin(zint) * self._cos(xint)
        self._matrix[0][2] = self._cos(zint) * self._sin(yint) * self._cos(
            xint
        ) + self._sin(zint) * self._sin(xint)
        self._matrix[1][0] = self._sin(zint) * self._cos(yint)
        self._matrix[1][1] = self._sin(zint) * self._sin(yint) * self._sin(
            xint
        ) + self._cos(zint) * self._cos(xint)
        self._matrix[1][2] = self._sin(zint) * self._sin(yint) * self._cos(
            xint
        ) - self._cos(zint) * self._sin(xint)
        self._matrix[2][0] = -self._sin(yint)
        self._matrix[2][1] = self._cos(yint) * self._sin(xint)
        self._matrix[2][2] = self._cos(yint) * self._cos(xint)

    @staticmethod
    def _cos(val: int) -> int:
        if val == 0:
            return 1
        elif val == 2:
            return -1
        else:
            return 0

    @staticmethod
    def _sin(val: int) -> int:
        if val == 1:
            return 1
        elif val == 3:
            return -1
        else:
            return 0

    @staticmethod
    def _float_rotation_to_discrete(rotation: float) -> int:
        if not np.isclose(rotation % (math.pi / 2.0), 0.0):
            raise NotImplementedError(
                "Angles in modular robot can currently only be multiples of half pi(90 degrees)."
            )

        # see rotation class. works with integers that are multiples of pi/2
        return ((int(round(rotation / (math.pi / 2.0))) % 4) + 4) % 4

    def __mul__(self, other: Union[_Rotation, Vector3]) -> Union[_Rotation, Vector3]:
        if isinstance(other, _Rotation):
            newrot = _Rotation(0, 0, 0)
            newrot._matrix = np.matmul(self._matrix, other._matrix)
            return newrot
        elif isinstance(other, Vector3):
            res = np.matmul(self._matrix, other)
            return Vector3([res[0], res[1], res[2]])
        else:
            raise NotImplementedError()

    def as_euler(self) -> Tuple[float, float, float]:
        return scipy.spatial.transform.Rotation.from_matrix(self._matrix).as_euler(
            "xyz"
        )


class _ModularRobotToSdf:
    @dataclass
    class _Link:
        name: str
        element: xml.Element

    @dataclass
    class _Joint:
        parent: _ModularRobotToSdf._Link
        child: _ModularRobotToSdf._Link

    _links: List[_Link]

    def __init__(self):
        self._links = []

    @staticmethod
    def _euler_from_quaternion(quaternion: Quaternion):
        """
        Quaternion to euler angles. xyz, right handed

        source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    @staticmethod
    def _make_pose(position: Vector3, orientation: _Rotation) -> xml.Element:
        pose = xml.Element("pose")
        pose.text = "{:e} {:e} {:e} {:e} {:e} {:e}".format(
            *position,
            *orientation.as_euler(),
        )
        return pose

    def modular_robot_to_sdf(
        self,
        robot: ModularRobot,
        model_name: str,
        position: Vector3,
        orientation: Quaternion,
    ) -> str:
        assert len(model_name) > 0, "model name must be at least one character."
        assert model_name[0].isalpha(), "model_name must start with a letter."
        assert not bool(
            regex.compile(r"[^a-zA-Z0-9_]").search(model_name)
        ), "model_name must consist only of letters, numbers, and underscores."

        # make basic sdf structure with model
        sdf = xml.Element("sdf", {"version": "1.6"})
        model = xml.SubElement(sdf, "model", {"name": model_name})
        model.append(
            _ModularRobotToSdf._make_pose(
                position, _Rotation(*self._euler_from_quaternion(orientation))
            )
        )

        core_link = xml.SubElement(model, "link", {"name": "core"})
        core_link.append(
            _ModularRobotToSdf._make_pose(
                Vector3([0.0, 0.0, 0.1]), _Rotation()
            )  # TODO remove z
        )
        xml.SubElement(core_link, "self_collide").text = "False"
        self._links.append(self._Link("core", core_link))
        self.make_module(
            robot.body.core,
            self._links[0],
            "core",
            Vector3([0.0, 0.0, 0.0]),
            _Rotation(),
        )

        return minidom.parseString(
            xml.tostring(sdf, encoding="unicode", method="xml")
        ).toprettyxml(indent="    ")

    def make_module(
        self,
        module: Module,
        link: _ModularRobotToSdf._Link,
        name_prefix: str,
        attachment_offset: Vector3,
        forward: _Rotation,
    ) -> None:
        if module.type == module.Type.CORE:
            self.make_core(cast(Core, module), link, name_prefix)
        elif module.type == module.Type.BRICK:
            self.make_brick(
                cast(Brick, module), link, name_prefix, attachment_offset, forward
            )
        elif module.type == module.type.ACTIVE_HINGE:
            self.make_active_hinge(
                cast(ActiveHinge, module), link, name_prefix, attachment_offset, forward
            )
        else:
            raise NotImplementedError("Module type not implemented")

    def make_module_directed(
        self,
        module: Module,
        link: _ModularRobotToSdf._Link,
        name_prefix: str,
        parent_position: Vector3,
        parent_orientation: _Rotation,
        radius: float,
        normal: _Rotation,
    ) -> None:
        rotation = parent_orientation * normal

        self.make_module(
            module,
            link,
            name_prefix,
            parent_position + rotation * Vector3([radius, 0.0, 0.0]),
            rotation,
        )

    def make_core(
        self, module: Core, link: _ModularRobotToSdf._Link, name_prefix: str
    ) -> None:
        sizexy = 0.089
        sizez = 0.045

        position = Vector3()
        orientation = _Rotation()

        self.add_visual(
            link.element,
            f"{name_prefix}_core_visual",
            position,
            orientation,
            (1.0, 1.0, 0.0, 1.0),
            "model://rg_robot/meshes/CoreComponent.dae",
        )

        self.add_box_collision(
            link.element,
            f"{name_prefix}_core_collision",
            position,
            orientation,
            (sizexy, sizexy, sizez),
        )

        if module.front is not None:
            self.make_module_directed(
                module.front.module,
                link,
                f"{name_prefix}_front",
                position,
                orientation,
                sizexy / 2.0,
                _Rotation(0, 0, 0) * _Rotation(module.front.rotation, 0, 0),
            )
        if module.back is not None:
            self.make_module_directed(
                module.back.module,
                link,
                f"{name_prefix}_back",
                position,
                orientation,
                sizexy / 2.0,
                _Rotation(0, 0, 2 * math.pi / 2.0)
                * _Rotation(module.back.rotation, 0, 0),
            )
        if module.left is not None:
            self.make_module_directed(
                module.left.module,
                link,
                f"{name_prefix}_left",
                position,
                orientation,
                sizexy / 2.0,
                _Rotation(0, 0, 1 * math.pi / 2.0)
                * _Rotation(module.left.rotation, 0, 0),
            )
        if module.right is not None:
            self.make_module_directed(
                module.right.module,
                link,
                f"{name_prefix}_right",
                position,
                orientation,
                sizexy / 2.0,
                _Rotation(0, 0, 3 * math.pi / 2.0)
                * _Rotation(module.right.rotation, 0, 0),
            )

    def make_brick(
        self,
        module: Brick,
        link: _ModularRobotToSdf._Link,
        name_prefix: str,
        attachment_offset: Vector3,
        orientation: _Rotation,
    ) -> None:
        sizexy = 0.041
        sizez = 0.0355

        position = attachment_offset + orientation * Vector3([sizexy / 2.0, 0.0, 0.0])

        self.add_visual(
            link.element,
            f"{name_prefix}_brick_visual",
            position,
            orientation,
            (1.0, 0.0, 0.0, 1.0),
            "model://rg_robot/meshes/FixedBrick.dae",
        )

        self.add_box_collision(
            link.element,
            f"{name_prefix}_brick_collision",
            position,
            orientation,
            (sizexy, sizexy, sizez),
        )

        if module.front is not None:
            self.make_module_directed(
                module.front.module,
                link,
                f"{name_prefix}_front",
                position,
                orientation,
                sizexy / 2.0,
                _Rotation(0, 0, 0) * _Rotation(module.front.rotation, 0, 0),
            )
        if module.back is not None:
            self.make_module_directed(
                module.back.module,
                link,
                f"{name_prefix}_back",
                position,
                orientation,
                sizexy / 2.0,
                _Rotation(0, 0, 2 * math.pi / 2.0)
                * _Rotation(module.back.rotation, 0, 0),
            )
        if module.left is not None:
            self.make_module_directed(
                module.left.module,
                link,
                f"{name_prefix}_left",
                position,
                orientation,
                sizexy / 2.0,
                _Rotation(0, 0, 1 * math.pi / 2.0)
                * _Rotation(module.left.rotation, 0, 0),
            )
        if module.right is not None:
            self.make_module_directed(
                module.right.module,
                link,
                f"{name_prefix}_right",
                position,
                orientation,
                sizexy / 2.0,
                _Rotation(0, 0, 3 * math.pi / 2.0)
                * _Rotation(module.right.rotation, 0, 0),
            )

    def make_active_hinge(
        self,
        module: ActiveHinge,
        link: _ModularRobotToSdf._Link,
        name_prefix: str,
        attachment_offset: Vector3,
        forward: _Rotation,
    ) -> None:
        pass

    def add_visual(
        self,
        element: xml.Element,
        name: str,
        position: Vector3,
        orientation: _Rotation,
        color: Tuple[float, float, float, float],
        model: str,
    ) -> None:
        visual = xml.SubElement(element, "visual", {"name": name})
        visual.append(self._make_pose(position, orientation))

        material = xml.SubElement(visual, "material")
        xml.SubElement(
            material, "ambient"
        ).text = f"{color[0]} {color[1]} {color[2]} {color[3]}"
        xml.SubElement(
            material, "diffuse"
        ).text = f"{color[0]} {color[1]} {color[2]} {color[3]}"
        xml.SubElement(material, "specular").text = "0.1 0.1 0.1 1.0"

        geometry = xml.SubElement(visual, "geometry")
        mesh = xml.SubElement(geometry, "mesh")
        xml.SubElement(mesh, "uri").text = model

    def add_box_collision(
        self,
        element: xml.Element,
        name: str,
        position: Vector3,
        orientation: _Rotation,
        box_size: Vector3,
    ):
        collision = xml.SubElement(element, "collision", {"name": name})
        collision.append(self._make_pose(position, orientation))

        surface = xml.SubElement(collision, "surface")

        contact = xml.SubElement(surface, "contact")
        ode = xml.SubElement(contact, "ode")
        xml.SubElement(ode, "kd").text = "{:e}".format(10000000.0 / 3.0)
        xml.SubElement(ode, "kp").text = "{:e}".format(90000)

        friction = xml.SubElement(surface, "friction")
        ode = xml.SubElement(friction, "ode")
        xml.SubElement(ode, "mu").text = "{:e}".format(1.0)
        xml.SubElement(ode, "mu2").text = "{:e}".format(1.0)
        xml.SubElement(ode, "slip1").text = "{:e}".format(0.01)
        xml.SubElement(ode, "slip2").text = "{:e}".format(0.01)
        bullet = xml.SubElement(friction, "bullet")
        xml.SubElement(bullet, "friction").text = "{:e}".format(1.0)
        xml.SubElement(bullet, "friction2").text = "{:e}".format(1.0)

        geometry = xml.SubElement(collision, "geometry")
        box = xml.SubElement(geometry, "box")
        xml.SubElement(box, "size").text = "{:e} {:e} {:e}".format(*box_size)
