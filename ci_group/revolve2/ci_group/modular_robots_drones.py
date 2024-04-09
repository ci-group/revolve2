import numpy as np

from revolve2.modular_robot.body.drone import MotorImpl, DroneBodyImpl
from pyrr import Quaternion, Vector3


def all() -> list[DroneBodyImpl]:
    """
    Get a list of all standard module robots.

    :returns: The list of robots.
    """
    return [random(), cross()]

def random() -> DroneBodyImpl:
    """
    Random drone, randomizing number of motors/arms and where 
    they are positioned and orientated in relation to the core.

    :returns: the robot.
    """
    body = DroneBodyImpl()

    num_arms = np.random.randint(0,16)
    def random_quaternion():
        u,v,w = np.random.uniform(0,1,3)
        q = [np.sqrt(1-u)*np.sin(np.pi*v), np.sqrt(1-u)*np.cos(np.pi*v), np.sqrt(u)*np.sin(np.pi*w), np.sqrt(u)*np.cos(np.pi*w)]
        return Quaternion(q)

    def random_position():
        pos = np.random.uniform(-1,1,3)
        while np.linalg.norm(pos) > 1: pos = np.random.uniform(-1,1,3)
        return pos
    
    for _ in range(num_arms):
        gear = 0.1*(np.random.randint(2)*2-1)
        body.core_v1.add_attachment(MotorImpl(position=Vector3(random_position()), orientation=random_quaternion(), gear=gear))

    return body

def cross() -> DroneBodyImpl:
    """
    Get the cross drone.
    0     0
     \   /
       _
      |_|
     /   \
    0     0
    :returns: the robot.
    """
    body = DroneBodyImpl()

    body.core_v1.add_attachment(MotorImpl(position=Vector3([-0.1,0.1,0]), orientation=Quaternion(), gear=-0.1))
    body.core_v1.add_attachment(MotorImpl(position=Vector3([0.1,0.1,0]), orientation=Quaternion(), gear=0.1))
    body.core_v1.add_attachment(MotorImpl(position=Vector3([-0.1,-0.1,0]), orientation=Quaternion(), gear=-0.1))
    body.core_v1.add_attachment(MotorImpl(position=Vector3([0.1,-0.1,0]), orientation=Quaternion(), gear=0.1))

    return body

def plus() -> DroneBodyImpl:
    """
    Get the plus drone.
        0  
        |
        _
    0--|_|--0
        |
        0
    :returns: the robot.
    """
    body = DroneBodyImpl()

    body.core_v1.add_attachment(MotorImpl(position=Vector3([0.0,0.1,0]), orientation=Quaternion(), gear=-0.1))
    body.core_v1.add_attachment(MotorImpl(position=Vector3([0.1,0.0,0]), orientation=Quaternion(), gear=0.1))
    body.core_v1.add_attachment(MotorImpl(position=Vector3([0.0,-0.1,0]), orientation=Quaternion(), gear=-0.1))
    body.core_v1.add_attachment(MotorImpl(position=Vector3([-0.1,0.0,0]), orientation=Quaternion(), gear=0.1))

    return body