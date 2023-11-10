import os
import tempfile
from typing import Any

import mujoco

try:
    import logging

    old_len = len(logging.root.handlers)

    from dm_control import mjcf

    new_len = len(logging.root.handlers)

    assert (
        old_len + 1 == new_len
    ), "dm_control not adding logging handler as expected. Maybe they fixed their annoying behaviour? https://github.com/deepmind/dm_control/issues/314"

    logging.root.removeHandler(logging.root.handlers[-1])
except Exception as e:
    print("Failed to fix absl logging bug", e)
    pass

from revolve2.simulation.scene import Scene, UUIDKey
from revolve2.simulation.scene.conversion import multi_body_system_to_urdf
from revolve2.simulation.scene.geometry import Geometry, GeometryHeightmap

from ._abstraction_to_mujoco_mapping import (
    AbstractionToMujocoMapping,
    JointHingeMujoco,
    MultiBodySystemMujoco,
)


def scene_to_model(
    scene: Scene,
    simulation_timestep: float,
    cast_shadows: bool,
    fast_sim: bool,
) -> tuple[mujoco.MjModel, AbstractionToMujocoMapping]:
    """
    Convert a scene to a MuJoCo model.

    :param cast_shadows: Whether shadows are cast by the light.
    :param scene: The scene to convert.
    :param simulation_timestep: The duration to integrate over during each step of the simulation. In seconds.
    :param fast_sim: If simulations have to be fast, unnecessary stuff will be turned off.
    :returns: The created MuJoCo model and mapping from the simulation abstraction to the model.
    """
    mapping = AbstractionToMujocoMapping()

    env_mjcf = mjcf.RootElement(model="scene")

    env_mjcf.compiler.angle = "radian"

    env_mjcf.option.timestep = simulation_timestep
    env_mjcf.option.integrator = "RK4"

    env_mjcf.option.gravity = [0, 0, -9.81]

    env_mjcf.worldbody.add(
        "light",
        pos=[0, 0, 100],
        ambient=[0.5, 0.5, 0.5],
        directional=True,
        castshadow=cast_shadows,
    )
    env_mjcf.visual.headlight.active = 0

    conversions = [
        multi_body_system_to_urdf(multi_body_system, f"mbs{multi_body_system_index}")
        for multi_body_system_index, multi_body_system in enumerate(
            scene.multi_body_systems
        )
    ]
    all_joints_and_names = [v for _, _, _, v, _ in conversions]

    heightmaps: list[GeometryHeightmap] = []

    for multi_body_system, (
        urdf,
        plane_geometries,
        heightmap_geometries,
        joints_and_names,
        geoms_and_names,
    ) in zip(
        scene.multi_body_systems,
        conversions,
        strict=True,
    ):
        multi_body_system_model = mujoco.MjModel.from_xml_string(urdf)

        # MuJoCo can only save to a file, not directly to string,
        # so we create a temporary file.
        try:
            with tempfile.NamedTemporaryFile(
                mode="r+", delete=True, suffix="_revolve2_mujoco.mjcf"
            ) as mjcf_file:
                mujoco.mj_saveLastXML(mjcf_file.name, multi_body_system_model)
                multi_body_system_mjcf = mjcf.from_file(mjcf_file)
        # handle an exception when the xml saving fails, it's almost certain to occur on Windows
        # since NamedTemporaryFile can't be opened twice when the file is still open.
        except Exception:
            with tempfile.NamedTemporaryFile(
                mode="r+", delete=False, suffix="_revolve2_mujoco.mjcf"
            ) as mjcf_file:
                # to make sure the temp file is always deleted,
                # an error catching is needed, in case the xml saving fails and crashes the program
                try:
                    mujoco.mj_saveLastXML(mjcf_file.name, multi_body_system_model)
                    multi_body_system_mjcf = mjcf.from_file(
                        mjcf_file,
                    )
                    # On Windows, an open file can’t be deleted, and hence it has to be closed first before removing
                    mjcf_file.close()
                    os.remove(mjcf_file.name)
                except Exception as e:
                    logging.info(repr(e))
                    # On Windows, an open file can’t be deleted, and hence it has to be closed first before removing
                    mjcf_file.close()
                    os.remove(mjcf_file.name)

        # The following few are set automatically during the urdf conversion,
        # but make no sense when we combine multiple URDFs.
        # So, we remote them and have mujoco calculate them for us.
        multi_body_system_mjcf.statistic.extent = None
        multi_body_system_mjcf.statistic.center = None
        multi_body_system_mjcf.statistic.meansize = None
        attachment_frame = env_mjcf.attach(multi_body_system_mjcf)
        attachment_frame.pos = [
            multi_body_system.pose.position.x,
            multi_body_system.pose.position.y,
            multi_body_system.pose.position.z,
        ]
        attachment_frame.quat = [
            multi_body_system.pose.orientation.x,
            multi_body_system.pose.orientation.y,
            multi_body_system.pose.orientation.z,
            multi_body_system.pose.orientation.w,
        ]
        if not multi_body_system.is_static:
            attachment_frame.add("freejoint")

        # Add actuation to joints
        for joint, name in joints_and_names:
            # Add rotor inertia to joints. This value is arbitrarily chosen and appears stable enough.
            # Fine-tuning the armature value might be needed later.
            multi_body_system_mjcf.find(
                namespace="joint", identifier=name
            ).armature = joint.armature
            multi_body_system_mjcf.actuator.add(
                "position",
                kp=joint.pid_gain_p,
                joint=multi_body_system_mjcf.find(
                    namespace="joint",
                    identifier=name,
                ),
                name=f"actuator_position_{name}",
            )
            multi_body_system_mjcf.actuator.add(
                "velocity",
                kv=joint.pid_gain_d,
                joint=multi_body_system_mjcf.find(namespace="joint", identifier=name),
                name=f"actuator_velocity_{name}",
            )

        # Add plane geometries
        i_plane = 0
        for plane in plane_geometries:
            name = f"heightmap_{i_plane}"
            plane_kwargs: dict[str, Any] = {}
            if fast_sim:
                plane_kwargs[
                    "rgba"
                ] = plane.texture.primary_color.to_normalized_rgba_list()
            else:
                plane_kwargs["material"] = f"{name}_material"
                __make_material(env_mjcf, name=name, element=plane)

            env_mjcf.worldbody.add(
                "geom",
                type="plane",
                pos=[
                    plane.pose.position.x,
                    plane.pose.position.y,
                    plane.pose.position.z,
                ],
                quat=[
                    plane.pose.orientation.x,
                    plane.pose.orientation.y,
                    plane.pose.orientation.z,
                    plane.pose.orientation.w,
                ],
                size=[plane.size.x / 2.0, plane.size.y / 2.0, 1.0],
                **plane_kwargs,
            )
            i_plane += 1

        # Add heightmap geometries
        i_heightmap = 0
        for heightmap in heightmap_geometries:
            env_mjcf.asset.add(
                "hfield",
                name=f"hfield_{i_heightmap}",
                nrow=len(heightmap.heights),
                ncol=len(heightmap.heights[0]),
                size=[
                    heightmap.size.x,
                    heightmap.size.y,
                    heightmap.size.z,
                    heightmap.base_thickness,
                ],
            )

            name = f"heightmap_{i_heightmap}"
            hm_kwargs: dict[str, Any] = {}
            if fast_sim:
                hm_kwargs[
                    "rgba"
                ] = heightmap.texture.primary_color.to_normalized_rgba_list()
            else:
                hm_kwargs["material"] = f"{name}_material"
                __make_material(env_mjcf, name=name, element=heightmap)

            env_mjcf.worldbody.add(
                "geom",
                type="hfield",
                hfield=f"hfield_{i_heightmap}",
                pos=[
                    heightmap.pose.position.x,
                    heightmap.pose.position.y,
                    heightmap.pose.position.z,
                ],
                quat=[
                    heightmap.pose.orientation.x,
                    heightmap.pose.orientation.y,
                    heightmap.pose.orientation.z,
                    heightmap.pose.orientation.w,
                ],
                **hm_kwargs,
            )

            heightmaps.append(heightmap)
            i_heightmap += 1

        # Set colors of geometries
        for geom, name in geoms_and_names:
            if fast_sim:
                multi_body_system_mjcf.find(
                    "geom", name
                ).rgba = geom.texture.primary_color.to_normalized_rgba_list()
            else:
                m_name = f"geom_{name}"
                __make_material(multi_body_system_mjcf, name=m_name, element=geom)
                multi_body_system_mjcf.find(
                    "geom", name
                ).material = f"{m_name}_material"

    xml = env_mjcf.to_xml_string()
    assert isinstance(xml, str)

    model = mujoco.MjModel.from_xml_string(xml)

    # set height map values
    heightmap_offset = 0
    for heightmap in heightmaps:
        for x in range(len(heightmap.heights)):
            for y in range(len(heightmap.heights[0])):
                model.hfield_data[y * len(heightmap.heights) + x] = heightmap.heights[
                    x
                ][y]
        heightmap_offset += len(heightmap.heights) * len(heightmap.heights[0])

    # Create map from hinge joints to their corresponding indices in the ctrl and position array
    for mbs_i, joints_and_names in enumerate(all_joints_and_names):
        for joint, name in joints_and_names:
            mapping.hinge_joint[UUIDKey(joint)] = JointHingeMujoco(
                id=model.joint(f"mbs{mbs_i}/{name}").id,
                ctrl_index_position=model.actuator(
                    f"mbs{mbs_i}/actuator_position_{name}"
                ).id,
                ctrl_index_velocity=model.actuator(
                    f"mbs{mbs_i}/actuator_velocity_{name}"
                ).id,
            )

    for mbs_i, multi_body_system in enumerate(scene.multi_body_systems):
        mapping.multi_body_system[UUIDKey(multi_body_system)] = MultiBodySystemMujoco(
            id=model.body(f"mbs{mbs_i}/").id
        )

    return (model, mapping)


def __make_material(env: mjcf.RootElement, name: str, element: Geometry) -> None:
    width, height = element.texture.size
    mat_kwargs = {}

    if element.texture.reference is not None:
        tex_kwargs = {
            k: v for k, v in element.texture.reference.__dict__.items() if v is not None
        }
        env.asset.add(
            "texture",
            **tex_kwargs,
            name=f"{name}_texture",
            type=element.texture.map_type.value,
            width=width,
            height=height,
            rgb1=element.texture.primary_color.to_normalized_rgb_list(),
            rgb2=element.texture.secondary_color.to_normalized_rgb_list(),
        )
        mat_kwargs["texture"] = f"{name}_texture"

    env.asset.add(
        "material",
        **mat_kwargs,
        name=f"{name}_material",
        rgba=element.texture.base_color.to_normalized_rgba_list(),
        texrepeat=element.texture.repeat,
        emission=element.texture.emission,
        specular=element.texture.specular,
        shininess=element.texture.shininess,
        reflectance=element.texture.reflectance,
    )
