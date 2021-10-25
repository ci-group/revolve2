import xml.dom.minidom as minidom
import xml.etree.ElementTree as xml
from typing import List, Mapping, Optional, Tuple, cast

from pyrr import Quaternion, Vector3
from revolve2.core.physics_robot import Joint, PhysicsRobot
from revolve2.core.physics_robot.rigid_body import RigidBody


def to_urdf(
    physics_robot: PhysicsRobot,
    name: str,
    position: Vector3,
    orientation: Quaternion,
) -> str:
    urdf = xml.Element("robot", {"name": name})

    tree: Mapping[str, List[Joint]] = {}  # parent to children
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

    for el in _make_node(root, tree, Vector3()):
        urdf.append(el)

    return minidom.parseString(
        xml.tostring(urdf, encoding="unicode", method="xml")
    ).toprettyxml(indent="    ")


def _make_node(
    node: RigidBody, tree: Mapping[str, List[Joint]], offset: Vector3
) -> List[xml.Element]:
    elements = []

    link = xml.Element("link", {"name": node.name})
    elements.append(link)

    for collision in node.collisions:
        el = xml.SubElement(link, "collision")
        geometry = xml.SubElement(el, "geometry")
        xml.SubElement(
            geometry,
            "box",
            {
                "size": f"{collision.bounding_box[0]} {collision.bounding_box[1]} {collision.bounding_box[2]}"
            },
        )
        xyz = collision.position + offset
        rpy = _quaternion_to_euler(collision.orientation)
        xml.SubElement(
            el,
            "origin",
            {
                "rpy": f"{rpy[0]} {rpy[1]} {rpy[2]}",
                "xyz": f"{xyz[0]} {xyz[1]} {xyz[2]}",
            },
        )

    if node.name in tree:
        for joint in tree[node.name]:
            el = xml.Element("joint", name=joint.name, type="revolute")
            xml.SubElement(el, "parent", {"link": node.name})
            xml.SubElement(el, "child", {"link": joint.body2.name})
            xyz = node.orientation.inverse * (joint.position + offset - node.position)
            rpy = _quaternion_to_euler(node.orientation.inverse * joint.orientation)
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
                    "lower": "-7.853982e-01",
                    "upper": "7.853982e-01",
                    "effort": "1.765800e-01",
                    "velocity": "5.235988e+00",
                },
            )
            elements.append(el)
            elements += _make_node(
                joint.body2,
                tree,
                joint.orientation.inverse * (joint.body2.position - joint.position),
            )

    # visual = xml.SubElement(link, "visual")
    # geometry = xml.SubElement(visual, "geometry")
    # xml.SubElement(geometry, "box", {"size": "1 1 1"})

    return elements


def _quaternion_to_euler(quaternion: Quaternion) -> Tuple[float, float, float]:
    import warnings

    from scipy.spatial.transform import Rotation

    with warnings.catch_warnings():
        warnings.simplefilter(
            "ignore", UserWarning
        )  # ignore gimbal lock warning. it is irrelevant for us.
        euler = Rotation.from_quat(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        ).as_euler("xyz")

    return (cast(float, euler[0]), cast(float, euler[1]), cast(float, euler[2]))
