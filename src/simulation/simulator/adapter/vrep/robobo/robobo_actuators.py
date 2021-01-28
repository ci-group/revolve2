from simulation.simulator.adapter.vrep.coppelia_simulation import CoppeliaSimulation


class RoboboActuators:

    speed_normalizer = 10

    def __init__(self, simulation: CoppeliaSimulation, name: str):
        self.simulation: CoppeliaSimulation = simulation
        self.name: str = name

        self.right_motor, self.left_motor, self.tilt_motor, self.pan_motor = None, None, None, None

        self._initialize()

    def _initialize(self):
        self.right_motor = self.simulation.get_object_handle('Right_Motor{}'.format(self.name))
        self.left_motor = self.simulation.get_object_handle('Left_Motor{}'.format(self.name))
        self.tilt_motor = self.simulation.get_object_handle('Tilt_Motor{}'.format(self.name))
        self.pan_motor = self.simulation.get_object_handle('Pan_Motor{}'.format(self.name))

    def drive(self, left_speed, right_speed):
        self.simulation.set_joint_velocity(self.left_motor, left_speed / self.speed_normalizer)
        self.simulation.set_joint_velocity(self.right_motor, right_speed / self.speed_normalizer)

    def move_stop(self, left, right, millis=500):
        self.drive(left, right)
        self.simulation.sleep(millis)
        self.drive(0, 0)

    def set_phone_pan(self, pan_position, pan_speed, samples=25):
        """
        Command the robot to move the smartphone holder in the horizontal (pan) axis.
        Arguments
        pan_position: Angle to position the pan at.
        pan_speed: Movement speed for the pan mechanism.
        """
        self.simulation.set_speed_position(self.pan_motor, pan_position, pan_speed, samples)

    def set_phone_tilt(self, tilt_position, tilt_speed, samples=25):
        """
        Command the robot to move the smartphone holder in the vertical (tilt) axis.
        Arguments
        tilt_position: Angle to position the tilt at.
        tilt_speed: Movement speed for the tilt mechanism.
        """
        self.simulation.set_speed_position(self.tilt_motor, tilt_position, tilt_speed, samples)

    def reset(self):
        self.drive(0, 0)
