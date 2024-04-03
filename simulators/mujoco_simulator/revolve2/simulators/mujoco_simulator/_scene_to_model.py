import os
import tempfile
from itertools import product
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

from revolve2.simulation.scene import JointHinge, Motor, RigidBody, Scene, UUIDKey
from revolve2.simulation.scene.conversion import multi_body_system_to_urdf
from revolve2.simulation.scene.geometry import (
    Geometry,
    GeometryHeightmap,
    GeometryPlane,
)

from ._abstraction_to_mujoco_mapping import (
    AbstractionToMujocoMapping,
    CameraSensorMujoco,
    IMUSensorMujoco,
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
    all_joints_and_names = [c[3] for c in conversions]
    all_rigid_bodies_and_names = [c[5] for c in conversions]

    heightmaps: list[GeometryHeightmap] = []
    for mbs_i, (
        multi_body_system,
        (
            urdf,
            plane_geometries,
            heightmap_geometries,
            joints_and_names,
            motors_and_names,
            geoms_and_names,
            rigid_bodies_and_names,
        ),
    ) in enumerate(zip(scene.multi_body_systems, conversions, strict=True)):
        multi_body_system_model = mujoco.MjModel.from_xml_string(urdf)
        multi_body_system_mjcf = _create_tmp_file(multi_body_system_model)

        # The following few are set automatically during the urdf conversion,
        # but make no sense when we combine multiple URDFs.
        # So, we remote them and have mujoco calculate them for us.
        multi_body_system_mjcf.statistic.extent = None
        multi_body_system_mjcf.statistic.center = None
        multi_body_system_mjcf.statistic.meansize = None
        attachment_frame = env_mjcf.attach(multi_body_system_mjcf)
        attachment_frame.pos = [*multi_body_system.pose.position]
        attachment_frame.quat = [*multi_body_system.pose.orientation]
        if not multi_body_system.is_static:
            attachment_frame.add("freejoint")

        # Add actuation to joints
        _add_joint_actuators(joints_and_names, multi_body_system_mjcf)

        # Add motors
        _add_motor_actuators(motors_and_names, multi_body_system_mjcf)

        # Add sensors
        _add_sensors(rigid_bodies_and_names, mbs_i, multi_body_system_mjcf, env_mjcf)

        # Add plane geometries
        _add_planes(plane_geometries, fast_sim, env_mjcf)

        # Add heightmap geometries
        hmps = _add_heightmaps(heightmap_geometries, fast_sim, env_mjcf)
        heightmaps.extend(hmps)

        # Set colors of geometries
        _set_colors_and_materials(
            geoms_and_names, multi_body_system_mjcf, fast_sim=fast_sim
        )

    xml = env_mjcf.to_xml_string()
    assert isinstance(xml, str)

    model = mujoco.MjModel.from_xml_string(xml)

    # set height map values
    _set_heightmap_values(heightmaps, model)

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

    # Create sensor maps
    _creat_sensor_maps(all_rigid_bodies_and_names, mapping, model)

    return model, mapping


def _create_tmp_file(multi_body_system_model: mujoco.MjModel) -> mjcf.RootElement:
    """
    Create a temporary file.

    MuJoCo can only save to a file, not directly to string,

    :param multi_body_system_model: The multi-body-system model.
    :return: The root element for the mujoco viewer.
    """
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
    return multi_body_system_mjcf


def _add_planes(
    plane_geometries: list[GeometryPlane], fast_sim: bool, env_mjcf: mjcf.RootElement
) -> None:
    """
    Add plane objects to the mujoco simulation.

    :param plane_geometries: The plane geometries.
    :param fast_sim: Whether fast sim is used.
    :param env_mjcf: The mujoco environment to simulate.
    """
    i_plane = 0
    for plane in plane_geometries:
        name = f"heightmap_{i_plane}"
        plane_kwargs: dict[str, Any] = {}
        if fast_sim:
            plane_kwargs["rgba"] = plane.texture.primary_color.to_normalized_rgba_list()
        else:
            plane_kwargs["material"] = f"{name}_material"
            __make_material(env_mjcf, name=name, element=plane)

        env_mjcf.worldbody.add(
            "geom",
            type="plane",
            pos=[*plane.pose.position],
            quat=[*plane.pose.orientation],
            size=[plane.size.x / 2.0, plane.size.y / 2.0, 1.0],
            **plane_kwargs,
        )
        i_plane += 1


def _add_heightmaps(
    heightmap_geometries: list[GeometryHeightmap],
    fast_sim: bool,
    env_mjcf: mjcf.RootElement,
) -> list[GeometryHeightmap]:
    """
    Add heightmap geometries to the model.

    :param heightmap_geometries: The heightmap geometries.
    :param fast_sim: If fast sim is used.
    :param env_mjcf: The mujoco root object.
    :return: A list of heightmap geometries.
    """
    heightmaps: list[GeometryHeightmap] = []

    i_heightmap = 0
    for heightmap in heightmap_geometries:
        env_mjcf.asset.add(
            "hfield",
            name=f"hfield_{i_heightmap}",
            nrow=len(heightmap.heights),
            ncol=len(heightmap.heights[0]),
            size=[
                *heightmap.size,
                heightmap.base_thickness,
            ],
        )

        name = f"heightmap_{i_heightmap}"
        hm_kwargs: dict[str, Any] = {}
        if fast_sim:
            hm_kwargs["rgba"] = (
                heightmap.texture.primary_color.to_normalized_rgba_list()
            )
        else:
            hm_kwargs["material"] = f"{name}_material"
            __make_material(env_mjcf, name=name, element=heightmap)

        env_mjcf.worldbody.add(
            "geom",
            type="hfield",
            hfield=f"hfield_{i_heightmap}",
            pos=[*heightmap.pose.position],
            quat=[*heightmap.pose.orientation],
            **hm_kwargs,
        )

        heightmaps.append(heightmap)
        i_heightmap += 1
    return heightmaps


def _add_sensors(
    rigid_bodies_and_names: list[tuple[RigidBody, str]],
    mbs_i: int,
    multi_body_system_mjcf: mjcf.RootElement,
    env_mjcf: mjcf.RootElement,
) -> None:
    """
    Add sensors to the model.

    :param rigid_bodies_and_names: The rigid bodies and names.
    :param mbs_i: The current index of the multi body system.
    :param multi_body_system_mjcf: The MBS in mujoco format.
    :param env_mjcf: The environment in mujoco format.
    """
    for rigid_body, name in rigid_bodies_and_names:
        if name == f"mbs{mbs_i}":
            rigid_body_mjcf = multi_body_system_mjcf.worldbody
        else:
            rigid_body_mjcf = multi_body_system_mjcf.find(
                namespace="body", identifier=name
            )

        """Here we add the IMU Sensors."""
        for imu_i, imu in enumerate(rigid_body.sensors.imu_sensors):
            site_name = f"{name}_site_imu_{imu_i}"
            rigid_body_mjcf.add(
                "site",
                name=site_name,
                pos=[*imu.pose.position],
                quat=[*imu.pose.orientation],
            )
            gyro_name = f"imu_gyro_{name}_{imu_i}"
            multi_body_system_mjcf.sensor.add("gyro", name=gyro_name, site=site_name)
            accelerometer_name = f"imu_accelerometer_{name}_{imu_i}"
            multi_body_system_mjcf.sensor.add(
                "accelerometer",
                name=accelerometer_name,
                site=site_name,
            )

        """Here we add camera Sensors."""
        for camera_i, camera in enumerate(rigid_body.sensors.camera_sensors):
            camera_name = f"camera_{name}_{camera_i+1}"
            env_mjcf.worldbody.add(
                "camera",
                name=camera_name,
                mode="fixed",
                xyaxes="0 -1 0 0 0 1",
                dclass=env_mjcf.full_identifier,
            )
            site_name = f"{name}_site_camera_{camera_i+1}"
            env_mjcf.worldbody.add(
                "site",
                name=site_name,
                pos=[*camera.pose.position],
                quat=[*camera.pose.orientation],
            )


def _add_joint_actuators(
    joints_and_names: list[tuple[JointHinge, str]],
    multi_body_system_mjcf: mjcf.RootElement,
) -> None:
    """
    Add actuation to the joints.

    :param joints_and_names: The joints.
    :param multi_body_system_mjcf: The multi body system.
    """
    for joint, name in joints_and_names:
        # Add rotor inertia to joints. This value is arbitrarily chosen and appears stable enough.
        # Fine-tuning the armature value might be needed later.
        multi_body_system_mjcf.find(namespace="joint", identifier=name).armature = (
            joint.armature
        )
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

def _add_motor_actuators(
    motors_and_names: list[tuple[Motor, str]],
    multi_body_system_mjcf: mjcf.RootElement,
) -> None:
    """
    Add actuation to the motors.

    :param motors_and_names: The joints.
    :param multi_body_system_mjcf: The multi body system.
    """
    for motor, name in motors_and_names:
        multi_body_system_mjcf.actuator.add(
            "motor",
            ctrllimited="true",
            ctrlrange=f"{motor.ctrlrange[0]} {motor.ctrlrange[1]}",
            gear=f"0 0 1 0 0 {motor.motor_gear}",
            site=multi_body_system_mjcf.find(
                namespace="site",
                identifier=name,
            ),
            name=f"actuator_motor_{name}",
        )

def _set_colors_and_materials(
    geoms_and_names: list[tuple[Geometry, str]],
    multi_body_system_mjcf: mjcf.RootElement,
    fast_sim: bool,
) -> None:
    """
    Set the colors and materials for the geometries contained in the model.

    :param geoms_and_names: The geometries with their respective names in the model.
    :param multi_body_system_mjcf: The mujoco model.
    :param fast_sim: Whether fast sim is used.
    """
    for geom, name in geoms_and_names:
        if fast_sim:
            multi_body_system_mjcf.find("geom", name).rgba = (
                geom.texture.primary_color.to_normalized_rgba_list()
            )
        else:
            m_name = f"geom_{name}"
            __make_material(multi_body_system_mjcf, name=m_name, element=geom)
            multi_body_system_mjcf.find("geom", name).material = f"{m_name}_material"


def _set_heightmap_values(
    heightmaps: list[GeometryHeightmap], model: mujoco.MjModel
) -> None:
    """
    Set the values for the heightmaps.

    :param heightmaps: The heightmaps.
    :param model: The mujoco model.
    """
    heightmap_offset = 0

    for heightmap in heightmaps:
        for x, y in product(
            range(len(heightmap.heights)), range(len(heightmap.heights[0]))
        ):
            model.hfield_data[y * len(heightmap.heights) + x] = heightmap.heights[x][y]
        heightmap_offset += len(heightmap.heights) * len(heightmap.heights[0])


def _creat_sensor_maps(
    all_rigid_bodies_and_names: list[list[tuple[RigidBody, str]]],
    mapping: AbstractionToMujocoMapping,
    model: mujoco.MjModel,
) -> None:
    """
    Create mappings of the imu sensors to the gyro and accelerator sensors in mujoco.

    :param all_rigid_bodies_and_names: The rigid bodies and their respective name in the model.
    :param mapping: The mapping of revolve2 components to Mujoco components.
    :param model: The mujoco model.
    """
    for mbs_i, rigid_bodies_and_names in enumerate(all_rigid_bodies_and_names):
        for rigid_body, name in rigid_bodies_and_names:
            for imu_i, imu in enumerate(rigid_body.sensors.imu_sensors):
                gyro_name = f"imu_gyro_{name}_{imu_i}"
                accelerometer_name = f"imu_accelerometer_{name}_{imu_i}"
                mapping.imu_sensor[UUIDKey(imu)] = IMUSensorMujoco(
                    gyro_id=model.sensor(f"mbs{mbs_i}/{gyro_name}").id,
                    accelerometer_id=model.sensor(
                        f"mbs{mbs_i}/{accelerometer_name}"
                    ).id,
                )

            for camera_i, camera in enumerate(rigid_body.sensors.camera_sensors):
                camera_name = f"camera_{name}_{camera_i+1}"
                mapping.camera_sensor[UUIDKey(camera)] = CameraSensorMujoco(
                    camera_id=model.camera(camera_name).id,
                    camera_size=camera.camera_size,
                )


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
