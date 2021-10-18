import xml.dom.minidom as minidom
import xml.etree.ElementTree as xml
from typing import Tuple, cast

from pyrr import Quaternion, Vector3, quaternion
from revolve2.core.modular_robot import ModularRobot
from revolve2.core.physics_robot import PhysicsRobot, collision


def modular_robot_to_sdf(
    robot: ModularRobot, model_name: str, position: Vector3, orientation: Quaternion
) -> str:
    physbot: PhysicsRobot = robot.to_physics_robot()

    sdf = xml.Element("sdf", {"version": "1.6"})
    model = xml.SubElement(sdf, "model", {"name": model_name})
    model.append(_make_pose(position, orientation))

    for body in physbot.bodies:
        link = xml.SubElement(model, "link", {"name": body.name})
        link.append(_make_pose(body.position, body.orientation))
        xml.SubElement(link, "self_collide").text = "True"

        for collision in body.collisions:
            link.append(
                _make_box_collision(
                    collision.name,
                    collision.position,
                    collision.orientation,
                    collision.bounding_box,
                )
            )

        for visual in body.visuals:
            link.append(
                _make_visual(
                    visual.name,
                    visual.position,
                    visual.orientation,
                    visual.color,
                    visual.model,
                )
            )

        mass = body.mass()

        inertia = body.inertia_tensor()
        center_of_mass = body.center_of_mass()

        inertial = xml.SubElement(link, "inertial")
        inertial.append(_make_pose(center_of_mass, Quaternion()))
        xml.SubElement(inertial, "mass").text = "{:e}".format(mass)
        inertia_el = xml.SubElement(inertial, "inertia")
        xml.SubElement(inertia_el, "ixx").text = "{:e}".format(inertia[0][0])
        xml.SubElement(inertia_el, "ixy").text = "{:e}".format(inertia[0][1])
        xml.SubElement(inertia_el, "ixz").text = "{:e}".format(inertia[0][2])
        xml.SubElement(inertia_el, "iyx").text = "{:e}".format(inertia[1][0])
        xml.SubElement(inertia_el, "iyy").text = "{:e}".format(inertia[1][1])
        xml.SubElement(inertia_el, "iyz").text = "{:e}".format(inertia[1][2])
        xml.SubElement(inertia_el, "izx").text = "{:e}".format(inertia[2][0])
        xml.SubElement(inertia_el, "izy").text = "{:e}".format(inertia[2][1])
        xml.SubElement(inertia_el, "izz").text = "{:e}".format(inertia[2][2])

    for joint in physbot.joints:
        el = xml.SubElement(
            model,
            "joint",
            {"name": f"{joint.name}", "type": "revolute"},
        )
        el.append(
            _make_pose(
                joint.body2.orientation.inverse
                * (joint.position - joint.body2.position),
                joint.body2.orientation.inverse * joint.orientation,
            )  # joint position & orientation are relative to child frame
        )
        xml.SubElement(el, "parent").text = joint.body1.name
        xml.SubElement(el, "child").text = joint.body2.name
        axis = xml.SubElement(el, "axis")
        xml.SubElement(axis, "xyz").text = "{:e} {:e} {:e}".format(0.0, 1.0, 0.0)
        xml.SubElement(axis, "use_parent_model_frame").text = "0"
        limit = xml.SubElement(axis, "limit")
        xml.SubElement(limit, "lower").text = "-7.853982e-01"
        xml.SubElement(limit, "upper").text = "7.853982e-01"
        xml.SubElement(limit, "effort").text = "1.765800e-01"
        xml.SubElement(limit, "velocity").text = "5.235988e+00"

    return minidom.parseString(
        xml.tostring(sdf, encoding="unicode", method="xml")
    ).toprettyxml(indent="    ")


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


def _make_pose(position: Vector3, orientation: Quaternion) -> xml.Element:
    pose = xml.Element("pose")
    pose.text = "{:e} {:e} {:e} {:e} {:e} {:e}".format(
        *position,
        *_quaternion_to_euler(orientation),
    )
    return pose


def _make_visual(
    name: str,
    position: Vector3,
    orientation: Quaternion,
    color: Tuple[float, float, float, float],
    model: str,
):
    visual = xml.Element("visual", {"name": name})
    visual.append(_make_pose(position, orientation))

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

    return visual


def _make_box_collision(
    name: str,
    position: Vector3,
    orientation: Quaternion,
    box_size: Vector3,
):
    collision = xml.Element("collision", {"name": name})
    collision.append(_make_pose(position, orientation))

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

    return collision
