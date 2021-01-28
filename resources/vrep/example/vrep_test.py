import sys

import numpy as np
from vrep.core import vrep


class VREPClient:

    def __init__(self):
        vrep.simxFinish(-1) # just in case, close all opened connections
        self.clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)  # start a connection
        self.number_of_limbs = 6
        self.number_of_joints = 3
        self.joint_handles = self._get_joints()
        print(self.joint_handles)
        if self.clientID != -1:
            print("Connected to remote API server")
        else:
            print("Not connected to remote API server")
            sys.exit("Could not connect")

    def _get_joints(self):
        joints = []
        for i in range(self.number_of_limbs):
            ret1, joint = vrep.simxGetObjectHandle(self.clientID, 'hexa_joint1_%d' % i, vrep.simx_opmode_oneshot_wait)
            if ret1 == 0:
                joints.append(joint)
        return joints

    def _getJointsPosition(self):
        print(self.joint_handles)
        return [vrep.simxGetJointPosition(self.clientID, self.joint_handles[i], vrep.simx_opmode_blocking)[1] for i in
                range(self.number_of_limbs)]

    def move(self, joint_angles):
        try:
            # 'np.allclose' returns True if two arrays are element-wise equal within a tolerance (1 degree)
            while not np.allclose(self._getJointsPosition(), joint_angles, atol=1):
                vrep.simxPauseCommunication(self.clientID, True)
                print("paused")
                for i in range(self.number_of_limbs):
                    vrep.simxSetJointPosition(self.clientID, self.joint_handles[i],
                                                        joint_angles[i],
                                                        vrep.simx_opmode_oneshot)
                print("set")
                vrep.simxPauseCommunication(self.clientID, False)
                vrep.simxSynchronousTrigger(self.clientID)  # move simulation ahead one time step
                print("ran")
        except:
            print('Unable set joint position')


if __name__ == "__main__":
    vrep_client = VREPClient()

    angles = []
    for i in range(vrep_client.number_of_limbs):
        angles.append(45.0)

    for i in range(5):
        print("current", vrep_client._getJointsPosition())
        print("next", angles)
        vrep_client.move(angles)

    exit(0)
