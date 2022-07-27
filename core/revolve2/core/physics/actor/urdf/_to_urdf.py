import warnings
import xml.dom.minidom as minidom
import xml.etree.ElementTree as xml
from typing import Dict, List, Optional, Tuple, cast

import scipy.spatial.transform
from pyrr import Quaternion, Vector3

from .._actor import Actor
from .._joint import Joint
from .._rigid_body import RigidBody


def to_urdf(
    physics_robot: Actor,
    name: str,
    position: Vector3,
    orientation: Quaternion,
) -> str:
    """
    Convert an actor to urdf.

    :param physics_robot: The actor to convert.
    :param name: Name to use for the robot in urdf.
    :param position: Position to use for the robot.
    :param orientation: Orientation to use for the robot.
    :returns: The created urdf.
    :raises RuntimeError: In case the robot cannot be converted to urdf.
    """
    urdf = xml.Element("robot", {"name": name})

    tree: Dict[str, List[Joint]] = {}  # parent to children
    seen_children = set()
    for joint in physics_robot.joints:
        if joint.body2.name in seen_children:
            raise RuntimeError(
                "Physics robot cannot be converted to urdf. Cannot be represented as tree structure. One or more joints has multiple parents."
            )
        seen_children.add(joint.body2.name)
        if joint.body1.name not in tree:
            tree[joint.body1.name] = []
        tree[joint.body1.name].append(joint)

    root: Optional[RigidBody] = None
    for body in physics_robot.bodies:
        if body.name not in seen_children:
            if root is not None:
                raise RuntimeError(
                    "Physics robot cannot be converted to urdf. Cannot be represented as tree structure. Some parts of the robot are detached; no single root body."
                )
            root = body
    if root is None:
        raise RuntimeError(
            "Physics robot cannot be converted to urdf. Require at least one body."
        )

    # pass negative position and inverse of orientation of robot to first element
    # so the robot will be positioned and rotated accordingly because the first link tries to compensate
    for el in _make_links(root, tree, -position, orientation.inverse):
        urdf.append(el)

    return cast(
        str,
        minidom.parseString(
            xml.tostring(urdf, encoding="unicode", method="xml")
        ).toprettyxml(indent="    "),
    )  # TODO for some reason the stubs for parseString have no proper return type. investigate


def _make_links(
    body: RigidBody,
    tree: Dict[str, List[Joint]],
    link_pos: Vector3,
    link_ori: Quaternion,
) -> List[xml.Element]:
    elements = []

    link = xml.Element("link", {"name": body.name})
    elements.append(link)

    com_xyz = link_ori.inverse * (
        body.position - link_pos + body.orientation * body.center_of_mass()
    )
    com_rpy = _quaternion_to_euler(link_ori.inverse * body.orientation)
    inertia = body.inertia_tensor()

    inertial = xml.SubElement(link, "inertial")
    xml.SubElement(
        inertial,
        "origin",
        {
            "rpy": f"{com_rpy[0]} {com_rpy[1]} {com_rpy[2]}",
            "xyz": f"{com_xyz[0]} {com_xyz[1]} {com_xyz[2]}",
        },
    )
    xml.SubElement(inertial, "mass", {"value": "{:e}".format(body.mass())})
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

    for collision in body.collisions:
        el = xml.SubElement(link, "collision", {"name": collision.name})
        geometry = xml.SubElement(el, "geometry")
        xml.SubElement(
            geometry,
            "box",
            {
                "size": f"{collision.bounding_box[0]} {collision.bounding_box[1]} {collision.bounding_box[2]}"
            },
        )
        xyz = link_ori.inverse * (
            body.position - link_pos + body.orientation * collision.position
        )
        rpy = _quaternion_to_euler(
            link_ori.inverse * body.orientation * collision.orientation
        )
        xml.SubElement(
            el,
            "origin",
            {
                "rpy": f"{rpy[0]} {rpy[1]} {rpy[2]}",
                "xyz": f"{xyz[0]} {xyz[1]} {xyz[2]}",
            },
        )

    if body.name in tree:
        for joint in tree[body.name]:
            el = xml.Element("joint", name=joint.name, type="revolute")
            xml.SubElement(el, "parent", {"link": body.name})
            xml.SubElement(el, "child", {"link": joint.body2.name})
            xyz = link_ori.inverse * (joint.position - link_pos)
            rpy = _quaternion_to_euler(link_ori.inverse * joint.orientation)
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
            elements += _make_links(
                joint.body2,
                tree,
                joint.position,
                joint.orientation,
            )

    # visual = xml.SubElement(link, "visual")
    # geometry = xml.SubElement(visual, "geometry")
    # xml.SubElement(geometry, "box", {"size": "1 1 1"})

    return elements


def _quaternion_to_euler(quaternion: Quaternion) -> Tuple[float, float, float]:
    with warnings.catch_warnings():
        warnings.simplefilter(
            "ignore", UserWarning
        )  # ignore gimbal lock warning. it is irrelevant for us.
        euler = scipy.spatial.transform.Rotation.from_quat(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        ).as_euler("xyz")

    return (cast(float, euler[0]), cast(float, euler[1]), cast(float, euler[2]))
