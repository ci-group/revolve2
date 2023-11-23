import uuid
import xml.dom.minidom as minidom
import xml.etree.ElementTree as xml

import numpy as np
from pyrr import Quaternion, Vector3

from .._joint_hinge import JointHinge
from .._multi_body_system import MultiBodySystem
from .._pose import Pose
from .._rigid_body import RigidBody
from ..geometry import Geometry, GeometryBox, GeometryHeightmap, GeometryPlane


def multi_body_system_to_urdf(
    multi_body_system: MultiBodySystem, name: str
) -> tuple[
    str,
    list[GeometryPlane],
    list[GeometryHeightmap],
    list[tuple[JointHinge, str]],
    list[tuple[Geometry, str]],
]:
    """
    Convert a multi-body system to URDF.

    It must be acyclic and the root body must also be the tree root.
    Furthermore, for each joint, the first body will be considered the parent, and as such must be the parent in the tree.

    Plane and heightmap cannot be represented in URDF and will be returned as in lists.
    It is checked they only exist as part of the root rigid body and for static multi-body systems.

    :param multi_body_system: The multi-body system to convert.
    :param name: The name to using in the URDF. It will be a prefix for every name in the model.
    :returns: A urdf string, plane geometries, heightmap geometries, joints and their names in the urdf, geometries and their names in the urdf
    :raises ValueError: In case the graph is cyclic.

    # noqa: DAR402 ValueError
    """
    return _URDFConverter().build(multi_body_system, name)


class _URDFConverter:
    base_name: str
    multi_body_system: MultiBodySystem
    visited_rigid_bodies: set[uuid.UUID]  # their indices
    joints_and_names: list[tuple[JointHinge, str]]
    geometries_and_names: list[tuple[Geometry, str]]
    planes: list[GeometryPlane]
    heightmaps: list[GeometryHeightmap]

    def build(
        self, multi_body_system: MultiBodySystem, name: str
    ) -> tuple[
        str,
        list[GeometryPlane],
        list[GeometryHeightmap],
        list[tuple[JointHinge, str]],
        list[tuple[Geometry, str]],
    ]:
        assert multi_body_system.has_root()

        self.multi_body_system = multi_body_system
        self.visited_rigid_bodies = set()
        self.joints_and_names = []
        self.geometries_and_names = []
        self.planes = []
        self.heightmaps = []

        urdf = xml.Element("robot", {"name": name})

        for element in self._make_links_xml_elements(
            multi_body_system.root,
            multi_body_system.root.initial_pose,
            f"{name}_root",
            parent_rigid_body=None,
        ):
            urdf.append(element)

        return (
            minidom.parseString(
                xml.tostring(urdf, encoding="unicode", method="xml")
            ).toprettyxml(indent="    "),
            self.planes,
            self.heightmaps,
            self.joints_and_names,
            self.geometries_and_names,
        )

    def _make_links_xml_elements(
        self,
        rigid_body: RigidBody,
        link_pose: Pose,
        rigid_body_name: str,
        parent_rigid_body: RigidBody | None,
    ) -> list[xml.Element]:
        if rigid_body.uuid in self.visited_rigid_bodies:
            raise ValueError("Multi-body system is cyclic.")
        self.visited_rigid_bodies.add(rigid_body.uuid)

        elements = []

        link = xml.Element("link", {"name": rigid_body_name})
        elements.append(link)

        com_xyz = link_pose.orientation.inverse * (
            rigid_body.initial_pose.position
            - link_pose.position
            + rigid_body.initial_pose.orientation * rigid_body.center_of_mass()
        )
        com_rpy = _quaternion_to_euler(
            link_pose.orientation.inverse * rigid_body.initial_pose.orientation
        )
        if rigid_body.mass() != 0.0:
            inertia = rigid_body.inertia_tensor()

            inertial = xml.SubElement(link, "inertial")
            xml.SubElement(
                inertial,
                "origin",
                {
                    "rpy": f"{com_rpy[0]} {com_rpy[1]} {com_rpy[2]}",
                    "xyz": f"{com_xyz[0]} {com_xyz[1]} {com_xyz[2]}",
                },
            )
            xml.SubElement(
                inertial, "mass", {"value": "{:e}".format(rigid_body.mass())}
            )
            xml.SubElement(
                inertial,
                "inertia",
                {
                    "ixx": "{:e}".format(inertia[0][0]),
                    "ixy": "{:e}".format(inertia[0][1]),
                    "ixz": "{:e}".format(inertia[0][2]),
                    "iyy": "{:e}".format(inertia[1][1]),
                    "iyz": "{:e}".format(inertia[1][2]),
                    "izz": "{:e}".format(inertia[2][2]),
                },
            )

        for geometry_index, geometry in enumerate(rigid_body.geometries):
            name = f"{rigid_body_name}_geom{geometry_index}"

            if isinstance(geometry, GeometryBox):
                self.geometries_and_names.append((geometry, name))
                self._add_geometry_box(
                    link=link,
                    name=name,
                    geometry=geometry,
                    link_pose=link_pose,
                    rigid_body=rigid_body,
                )
            elif isinstance(geometry, GeometryPlane):
                if parent_rigid_body is not None:
                    raise ValueError(
                        "Plane geometry can only be included in the root rigid body."
                    )
                if not self.multi_body_system.is_static:
                    raise ValueError(
                        "Plane geometry can only be included in static multi-body systems."
                    )
                self.planes.append(geometry)
            elif isinstance(geometry, GeometryHeightmap):
                if parent_rigid_body is not None:
                    raise ValueError(
                        "Heightmap geometry can only be included in the root rigid body."
                    )
                if not self.multi_body_system.is_static:
                    raise ValueError(
                        "Heightmap geometry can only be included in static multi-body systems."
                    )
                self.heightmaps.append(geometry)
            else:
                raise ValueError("Geometry not yet supported.")

        for joint_index, joint in enumerate(
            self.multi_body_system.get_joints_for_rigid_body(rigid_body)
        ):
            # Make sure we don't go back up the joint we came from.
            if parent_rigid_body is not None and (
                joint.rigid_body1.uuid == parent_rigid_body.uuid
                or joint.rigid_body2.uuid == parent_rigid_body.uuid
            ):
                continue

            if not isinstance(joint, JointHinge):
                raise ValueError(
                    "Joints other that hinge joints are not yet supported."
                )

            child_name = f"{rigid_body_name}_link{joint_index}"

            joint_name = f"{rigid_body_name}_joint{joint_index}"
            self.joints_and_names.append((joint, joint_name))
            el = xml.Element("joint", type="revolute", name=joint_name)
            xml.SubElement(
                el,
                "parent",
                {
                    "link": rigid_body_name
                    if joint.rigid_body1.uuid == rigid_body.uuid
                    else child_name
                },
            )
            xml.SubElement(
                el,
                "child",
                {
                    "link": rigid_body_name
                    if joint.rigid_body1.uuid != rigid_body.uuid
                    else child_name
                },
            )
            xyz = link_pose.orientation.inverse * (
                joint.pose.position - link_pose.position
            )
            rpy = _quaternion_to_euler(
                link_pose.orientation.inverse * joint.pose.orientation
            )
            xml.SubElement(
                el,
                "origin",
                {
                    "rpy": f"{rpy[0]} {rpy[1]} {rpy[2]}",
                    "xyz": f"{xyz[0]} {xyz[1]} {xyz[2]}",
                },
            )
            xml.SubElement(el, "axis", {"xyz": "0 1 0"})
            xml.SubElement(
                el,
                "limit",
                {
                    "lower": f"{-joint.range}",
                    "upper": f"{joint.range}",
                    "effort": f"{joint.effort}",
                    "velocity": f"{joint.velocity}",
                },
            )
            elements.append(el)
            elements += self._make_links_xml_elements(
                joint.rigid_body2,
                joint.pose,
                child_name,
                parent_rigid_body=rigid_body,
            )

        return elements

    def _add_geometry_box(
        self,
        link: xml.Element,
        name: str,
        geometry: GeometryBox,
        link_pose: Pose,
        rigid_body: RigidBody,
    ) -> None:
        el = xml.SubElement(link, "collision", {"name": name})
        geometry_xml = xml.SubElement(el, "geometry")
        xml.SubElement(
            geometry_xml,
            "box",
            {
                "size": f"{geometry.aabb.size.x} {geometry.aabb.size.y} {geometry.aabb.size.z}"
            },
        )
        xyz = link_pose.orientation.inverse * (
            rigid_body.initial_pose.position
            - link_pose.position
            + rigid_body.initial_pose.orientation * geometry.pose.position
        )
        rpy = _quaternion_to_euler(
            link_pose.orientation.inverse
            * rigid_body.initial_pose.orientation
            * geometry.pose.orientation
        )
        xml.SubElement(
            el,
            "origin",
            {
                "rpy": f"{rpy[0]} {rpy[1]} {rpy[2]}",
                "xyz": f"{xyz[0]} {xyz[1]} {xyz[2]}",
            },
        )

    def _add_geometry_plane(
        self,
        link: xml.Element,
        name: str,
        geometry: GeometryPlane,
        link_pose: Pose,
        rigid_body: RigidBody,
    ) -> None:
        PLANE_BOX_HEIGHT = 0.1

        el = xml.SubElement(link, "collision", {"name": name})
        geometry_xml = xml.SubElement(el, "geometry")
        xml.SubElement(
            geometry_xml,
            "box",
            {"size": f"{geometry.size.x} {geometry.size.y} {PLANE_BOX_HEIGHT}"},
        )
        xyz = link_pose.orientation.inverse * (
            rigid_body.initial_pose.position
            - link_pose.position
            + rigid_body.initial_pose.orientation
            * (geometry.pose.position + Vector3([0.0, 0.0, -PLANE_BOX_HEIGHT / 2.0]))
        )
        rpy = _quaternion_to_euler(
            link_pose.orientation.inverse
            * rigid_body.initial_pose.orientation
            * geometry.pose.orientation
        )
        xml.SubElement(
            el,
            "origin",
            {
                "rpy": f"{rpy[0]} {rpy[1]} {rpy[2]}",
                "xyz": f"{xyz[0]} {xyz[1]} {xyz[2]}",
            },
        )


def _quaternion_to_euler(quaternion: Quaternion) -> Vector3:
    """
    Convert a Quaternion to euler angles.

    If you want to see the maths behind this check the following page: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles .

    :param quaternion: The quaternion to convert.
    :return: Euler angles in form of a Vector3 (roll, pitch, yaw).
    """
    x, y, z, w = quaternion

    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = (
        2
        * np.arctan2(np.sqrt(1 + 2 * (w * y - x * z)), np.sqrt(1 - 2 * (w * y - x * z)))
        - np.pi / 2
    )
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return Vector3([roll, pitch, yaw])
