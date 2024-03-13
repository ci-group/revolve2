from scene._multi_body_system import MultiBodySystem
from scene.conversion import multi_body_system_to_urdf
from scene._pose import Pose
from pyrr import Quaternion, Vector3
from scene.vector2 import Vector2

from scene._rigid_body import RigidBody
from scene._joint_hinge import JointHinge
from scene.geometry import Geometry
from scene.geometry.textures import Texture
from scene.geometry import Geometry, GeometryBox, GeometryHeightmap, GeometryPlane, GeometryCylinder
from scene._aabb import AABB

if __name__ == "__main__":

    pose1 = Pose(position=Vector3([0.,0.,0.]), orientation=Quaternion())
    pose2 = Pose(position=Vector3([1.,0.,0.]), orientation=Quaternion())
    pose3 = Pose(position=Vector3([0.5,0.,0.]), orientation=Quaternion())

    plane = GeometryPlane(pose=pose1, mass=0.0, size=Vector2([20.0, 20.0]))
    geom1 = GeometryBox(pose=pose1, mass=0.0, texture=Texture(), aabb=AABB(size=Vector3([0.5,0.5,0.5])))
    geom2 = GeometryBox(pose=pose2, mass=0.0, texture=Texture(), aabb=AABB(size=Vector3([0.5,0.5,0.5])))
    plane_body = RigidBody(initial_pose=pose1, static_friction=1.0, dynamic_friction=1.0, geometries=[plane])
    geom1_body = RigidBody(initial_pose=pose1, static_friction=1.0, dynamic_friction=1.0, geometries=[geom1])
    geom2_body = RigidBody(initial_pose=pose2, static_friction=1.0, dynamic_friction=1.0, geometries=[geom2])
    joint = JointHinge(pose=pose3, rigid_body1=geom1_body, rigid_body2=geom2_body , axis=Vector3([0.5,0.5,0.5]), range=1.0, effort=0.1, velocity=0.0, armature=0.0, pid_gain_p=1.0, pid_gain_d=0.1)
    
    plane_multi_body_sys = MultiBodySystem(pose=pose1, is_static=True)
    plane_multi_body_sys.add_rigid_body(plane_body)

    multi_body_sys1 = MultiBodySystem(pose=pose1, is_static=False)
    multi_body_sys1.add_rigid_body(geom1_body)
    multi_body_sys1.add_rigid_body(geom2_body)
    multi_body_sys1.add_joint(joint)
    
    elements = multi_body_system_to_urdf(multi_body_sys1, name="robot")

    print(elements[0])
    # xml file,
    # self.planes,
    # self.heightmaps,
    # self.joints_and_names,
    # self.geometries_and_names,