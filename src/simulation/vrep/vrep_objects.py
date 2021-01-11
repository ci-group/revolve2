from vrep.core import vrep

print('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

if clientID != -1:
    print('Connected to remote API server')
    res,objs = vrep.simxGetObjects(clientID, vrep.sim_handle_all, vrep.simx_opmode_oneshot_wait)
    print(res, objs)
    if res == vrep.simx_return_ok:
        print('Number of objects in the scene: ', len(objs))
        ret1, ur10joint = vrep.simxGetObjectHandle(clientID, 'Ant_joint3Leg1', vrep.simx_opmode_oneshot_wait)
        ret2 = vrep.simxSetJointTargetPosition(clientID, ur10joint, 1.571, vrep.simx_opmode_oneshot)
        print(ret1, ur10joint, ret2)
    else:
        print('Remote API function call returned with error code: ', res)
    vrep.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
