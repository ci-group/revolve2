import random
import threading


class StoppableThread(threading.Thread):
    """Thread class with a stop() method. The thread itself has to check
        regularly for the stopped() condition."""

    def __init__(self):
        super(StoppableThread, self).__init__()
        self._stop_event = threading.Event()

    def stop(self):
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()


class Prey(StoppableThread):
    def __init__(self, robot, seed=42, log=None, level=2):
        super(Prey, self).__init__()
        self._log = log
        self._robot = robot
        # seed for the random function -> make reproducible the experiment
        self._seed = seed
        # default level is 2 -> medium
        self._level = level

    def _sensor_better_reading(self, sensors_values):
        """
        Normalising simulation sensor reading due to reuse old code
        :param sensors_values:
        :return:
        """
        old_min = 0
        old_max = 0.20
        new_min = 20000
        new_max = 0
        return [0 if value is False else (((value - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min for value in sensors_values]

    def run(self):
        """
        Method that moves the robot.
        It avoids obstacles and with a predefined probability it changes direction
        :return:
        """
        if self._log is not None:
            self._log.info("The robot prey is moving")
        else:
            print("The robot prey is moving")
        if self._level == 0:
            if self._log is not None:
                self._log.info("Level {} selected -> super easy option".format(self._level))
            else:
                print("Level {} selected -> super easy option".format(self._level))
            maximum_speed = 5.0
            turning_speed = 5.0
            epsilon = 0.02
        elif self._level == 1:
            if self._log is not None:
                self._log.info("Level {} selected -> easy option".format(self._level))
            else:
                print("Level {} selected -> easy option".format(self._level))
            maximum_speed = 10.0
            turning_speed = 10.0
            epsilon = 0.03
        elif self._level == 2:
            if self._log is not None:
                self._log.info("Level {} selected -> medium option".format(self._level))
            else:
                print("Level {} selected -> medium option".format(self._level))
            maximum_speed = 20.0
            turning_speed = 10.0
            epsilon = 0.06
        elif self._level == 3:
            if self._log is not None:
                self._log.info("Level {} selected -> hard option".format(self._level))
            else:
                print("Level {} selected -> hard option".format(self._level))
            maximum_speed = 40.0
            turning_speed = 20.0
            epsilon = 0.08
        elif self._level == 4:
            if self._log is not None:
                self._log.info("Level {} selected -> insane (Good luck Catching Me)".format(self._level))
            else:
                print("Level {} selected -> insane (Good luck Catching Me)".format(self._level))
            maximum_speed = 70.0
            turning_speed = 30.0
            epsilon = 0.1
        else:
            raise Exception("Level Value not correct, try from 0 to 4.")

        random.seed = self._seed

        while not self.stopped():
            speed_right = speed_left = maximum_speed
            if random.random() <= epsilon:
                speed_right = random.uniform(-turning_speed, turning_speed)
                speed_left = random.uniform(-turning_speed, turning_speed)
                for i in range(3):
                    self._robot.move(left=speed_right, right=speed_left, millis=200)
            self._robot.move(left=speed_right, right=speed_left, millis=200)
            sensors = self._sensor_better_reading(self._robot.read_irs())
            # self.log.debug(sensors)
            # print(sensors)
            if sum(sensors) != 0:
                index_max_value = sensors.index(max(sensors))
                if index_max_value == 5:
                    # central -> turn
                    if random.random() <= 0.5:
                        while sum(sensors[3:5]) > 500:
                            self._robot.move(left=-10.0, right=20.0, millis=500)
                            sensors = self._sensor_better_reading(self._robot.read_irs())
                    else:
                        while sum(sensors[6:8]) > 500:
                            self._robot.move(left=20.0, right=-10, millis=500)
                            sensors = self._sensor_better_reading(self._robot.read_irs())
                elif index_max_value == 3 or index_max_value == 4:
                    # front right right -> go left
                    while sum(sensors[3:5]) > 500:
                        self._robot.move(left=-10.0, right=20.0, millis=200)
                        sensors = self._sensor_better_reading(self._robot.read_irs())
                elif index_max_value == 7 or index_max_value == 6:
                    # front left left -> go right
                    while sum(sensors[6:8]) > 500:
                        self._robot.move(left=20.0, right=-10.0, millis=200)
                        sensors = self._sensor_better_reading(self._robot.read_irs())