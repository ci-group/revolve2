import os
import time

from abstract.sequential_identifier import SequentialIdentifier
from resources import vrep
from sys import platform
import subprocess


class CoppeliaCommunicationError(Exception):
    pass


class CoppeliaSimulator:

    port_identifier = SequentialIdentifier(19997)

    def __init__(self, coppelia_path: str, scene_path: str, headless: bool = False):
        self.client_id: int = -1
        self.coppelia_path: str = coppelia_path
        self.scene_path: str = scene_path
        self.headless: bool = headless
        self.port = self.port_identifier.id()

        self.client_process = None

    def launch(self):
        headless_argument = "-h" if self.headless else ""
        if platform == "linux" or platform == "linux2":
            # Linux: ./coppeliaSim.sh -h -s5000 -q myScene.ttt
            self.client_process = subprocess.Popen([self.coppelia_path + "/coppeliaSim.sh", headless_argument, "-q ", self.scene_path], shell=True)
        elif platform == "darwin":
            # Mac: ./coppeliaSim.app /Contents /MacOS /coppeliaSim -h -s5000 -q ../Resources/myScene.ttt
            self.client_process = subprocess.Popen([self.coppelia_path + "/coppeliaSim.app", "/Contents", "/MacOS", "/coppeliaSim", headless_argument, "-q", self.scene_path], shell=True)
        elif platform == "win32" or platform == "windows":
            # Windows: coppeliaSim.exe -h -s5000 -q myScene.ttt
            print([self.coppelia_path + "\\coppeliaSim.exe", "-gREMOTEAPISERVERSERVICE_" + str(self.port) + "_FALSE_TRUE", headless_argument, "-q", self.scene_path])
            self.client_process = subprocess.Popen('start ' + self.coppelia_path + "/coppeliaSim.exe -gREMOTEAPISERVERSERVICE_" + str(self.port) + "_FALSE_TRUE " + headless_argument + "-q " + self.scene_path, shell=True)
        else:
            print("Unrecognized operating system")
        print(self.client_process)
        print("Launched ", self.client_process)

    def connect(self, address='127.0.0.1'):
        self.launch()
        retries = 15
        connected = False

        for i in range(retries):
            # vrep.simxFinish(-1)  # just in case, close all opened connections
            self.client_id = vrep.simxStart(address, self.port, True, True, 1000, 5)  # Connect to V-REP
            if self.client_id >= 0:  # and clientID_0 != -1:
                self.wait_for_ping()
                print('Connected to remote API server: client id {}'.format(self.client_id))
                connected = True
                break
            else:
                print("Unable to connect, retrying... (%d / %d)" % (i+1, retries))

        if not connected:
            raise CoppeliaCommunicationError('Failed connecting to remote API server')

        return self

    def disconnect(self):
        self.client_process.terminate()
        vrep.unwrap_vrep(
            vrep.simxFinish(self.client_id)
        )

    def pause_simulation(self):
        vrep.unwrap_vrep(
            vrep.simxPauseSimulation(self.client_id, vrep.simx_opmode_blocking)
        )

    def play_simulation(self):
        vrep.unwrap_vrep(
            vrep.simxStartSimulation(self.client_id, vrep.simx_opmode_blocking)
        )
        self.wait_for_ping()

    def stop_world(self):
        vrep.unwrap_vrep(
            vrep.simxStopSimulation(self.client_id, vrep.simx_opmode_blocking)
        )
        self.wait_for_ping()

    def check_simulation_state(self):
        self.wait_for_ping()
        return vrep.unwrap_vrep(
            vrep.simxGetInMessageInfo(self.client_id, vrep.simx_headeroffset_server_state),
            ignore_novalue_error=True
        )

    def is_simulation_stopped(self):
        return not self.is_simulation_running()

    def is_simulation_running(self):
        info = self.check_simulation_state()
        return info & 1

    def wait_for_stop(self):
        """
        This function busy waits until the simulation is stopped
        """
        while self.is_simulation_running():
            pass

    def wait_for_ping(self, timeout_seconds=120.0):
        start_time = time.time()
        while time.time() - start_time < timeout_seconds:
            try:
                self._vrep_get_ping_time()
                # print("check success")
                return True
            except vrep.VrepApiError as _e:
                # print("check failed")
                time.sleep(0.05)

        print("{} seconds passed with ping not coming online, you may expericence problems with the connection".format(
            timeout_seconds))
        return False

    def _vrep_get_ping_time(self):
        return vrep.unwrap_vrep(vrep.simxGetPingTime(self.client_id))

    def get_simulation_time(self):
        """
        Gets the simulation time. Returns zero if the simulation is stopped.
        :return: simulation time in milliseconds.
        """
        self.wait_for_ping()
        return vrep.simxGetLastCmdTime(self.client_id)


if __name__ == "__main__":
    coppelia_path = "start C:\\\"Program Files\"\\CoppeliaRobotics\\CoppeliaSimEdu"
    scene_path = "C:\\Users\\daan_\\Documents\\GitHub\\revolve2\\resources\\vrep\scenes\\Robobo_Scene.ttt"
    s = CoppeliaSimulator(coppelia_path, scene_path, headless=True)
    s.connect()
