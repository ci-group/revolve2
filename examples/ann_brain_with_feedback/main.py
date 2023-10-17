"""Main script for the example."""

import numpy as np
import numpy.typing as npt

from revolve2.ci_group import modular_robots_v1, terrains
from revolve2.ci_group.simulation import make_standard_batch_parameters
from revolve2.experimentation.logging import setup_logging
from revolve2.experimentation.rng import make_rng_time_seed
from revolve2.modular_robot import ModularRobot, ModularRobotControlInterface
from revolve2.modular_robot.body.base import ActiveHinge, ActiveHingeSensor
from revolve2.modular_robot.brain import Brain, BrainInstance
from revolve2.modular_robot.sensor_state import ModularRobotSensorState
from revolve2.modular_robot_simulation import ModularRobotScene, simulate_scenes
from revolve2.simulators.mujoco_simulator import LocalSimulator


class ANNBrainInstance(BrainInstance):
    """ANN brain instance."""

    active_hinges: list[ActiveHinge]
    hidden_layer_width: int
    weights1: npt.NDArray[np.float_]
    biasses1: npt.NDArray[np.float_]
    weights2: npt.NDArray[np.float_]
    biasses2: npt.NDArray[np.float_]

    def __init__(
        self,
        active_hinges: list[ActiveHinge],
        hidden_layer_width: int,
        weights1: npt.NDArray[np.float_],
        biasses1: npt.NDArray[np.float_],
        weights2: npt.NDArray[np.float_],
        biasses2: npt.NDArray[np.float_],
    ) -> None:
        """
        Initialize the Object.

        :param active_hinges: The active hinges to control.
        :param hidden_layer_width: The number of neurons in the hidden layer.
        :param weights1: The weights for the connections from the input layer to the hidden layer. Shape: (len(active_hinges), hidden_layer_width).
        :param biasses1: The biasses for the hidden layer. Shape: (hidden_layer_width,).
        :param weights2: The weights for the connections from the hidden layer to the output layer. Shape: (hidden_layer_width, len(active_hinges)).
        :param biasses2: The biasses for the output layer. Shape: (len(active_hinges),).
        """
        self.active_hinges = active_hinges
        self.hidden_layer_width = hidden_layer_width
        self.weights1 = weights1
        self.biasses1 = biasses1
        self.weights2 = weights2
        self.biasses2 = biasses2

    @staticmethod
    def relu(x: npt.NDArray[np.float_]) -> npt.NDArray[np.float_]:
        """
        Calculat ReLU for the given array.

        :param x: The array.
        :returns: The array with ReLU applied.
        """
        return np.maximum(0, x)

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        """
        Control the modular robot.

        :param dt: Elapsed seconds since last call to this function.
        :param sensor_state: Interface for reading the current sensor state.
        :param control_interface: Interface for controlling the robot.
        """
        sensors = [
            active_hinge.sensor
            for active_hinge in self.active_hinges
            if active_hinge.sensor is not None
        ]
        assert len(sensors) == len(
            self.active_hinges
        ), "One of the active hinges does not have a sensor set."
        position_sensor_states = [
            sensor_state.get_active_hinge_sensor_state(sensor) for sensor in sensors
        ]

        input_layer = np.array([state.position for state in position_sensor_states])
        hidden_layer = self.relu(np.dot(input_layer, self.weights1) + self.biasses1)
        output_layer = np.tanh(np.dot(hidden_layer, self.weights2) + self.biasses2)

        for active_hinge, target in zip(self.active_hinges, output_layer, strict=True):
            control_interface.set_active_hinge_target(active_hinge, target)


class ANNBrain(Brain):
    """The ANN brain."""

    active_hinges: list[ActiveHinge]
    hidden_layer_width: int
    weights1: npt.NDArray[np.float_]
    biasses1: npt.NDArray[np.float_]
    weights2: npt.NDArray[np.float_]
    biasses2: npt.NDArray[np.float_]

    def __init__(
        self,
        active_hinges: list[ActiveHinge],
        hidden_layer_width: int,
        weights1: npt.NDArray[np.float_],
        biasses1: npt.NDArray[np.float_],
        weights2: npt.NDArray[np.float_],
        biasses2: npt.NDArray[np.float_],
    ) -> None:
        """
        Initialize the Object.

        :param active_hinges: The active hinges to control.
        :param hidden_layer_width: The number of neurons in the hidden layer.
        :param weights1: The weights for the connections from the input layer to the hidden layer. Shape: (len(active_hinges), hidden_layer_width).
        :param biasses1: The biasses for the hidden layer. Shape: (hidden_layer_width,).
        :param weights2: The weights for the connections from the hidden layer to the output layer. Shape: (hidden_layer_width, len(active_hinges)).
        :param biasses2: The biasses for the output layer. Shape: (len(active_hinges),).
        """
        self.active_hinges = active_hinges
        self.hidden_layer_width = hidden_layer_width
        self.weights1 = weights1
        self.biasses1 = biasses1
        self.weights2 = weights2
        self.biasses2 = biasses2

    def make_instance(self) -> BrainInstance:
        """
        Create an instance of this brain.

        :returns: The created instance.
        """
        return ANNBrainInstance(
            active_hinges=self.active_hinges,
            hidden_layer_width=self.hidden_layer_width,
            weights1=self.weights1,
            biasses1=self.biasses1,
            weights2=self.weights2,
            biasses2=self.biasses2,
        )


def main() -> None:
    """Run the simulation."""
    # Set up logging.
    setup_logging()

    # Set up the random number generator.
    rng = make_rng_time_seed()

    # Create a body for the robot.
    body = modular_robots_v1.gecko_v1()

    # Add sensors to each active hinge that measure the current angle of the hinge.
    active_hinges = body.find_active_hinges()
    for active_hinge in active_hinges:
        active_hinge.sensor = ActiveHingeSensor()

    # Create a brain for the robot.
    active_hinges = body.find_active_hinges()
    hidden_layer_width = len(active_hinges)
    weights1 = rng.random(size=(len(active_hinges), hidden_layer_width)) * 2.0 - 1.0
    biasses1 = rng.random(size=hidden_layer_width) * 2.0 - 1.0
    weights2 = rng.random(size=(hidden_layer_width, len(active_hinges))) * 2.0 - 1.0
    biasses2 = rng.random(size=len(active_hinges)) * 2.0 - 1.0
    brain = ANNBrain(
        active_hinges=active_hinges,
        hidden_layer_width=hidden_layer_width,
        weights1=weights1,
        biasses1=biasses1,
        weights2=weights2,
        biasses2=biasses2,
    )

    # Combine the body and brain into a modular robot.
    robot = ModularRobot(body, brain)

    # Create the scene.
    scene = ModularRobotScene(terrain=terrains.flat())
    scene.add_robot(robot)

    # Simulate the scene.
    simulator = LocalSimulator()
    simulate_scenes(
        simulator=simulator,
        batch_parameters=make_standard_batch_parameters(),
        scenes=scene,
    )


if __name__ == "__main__":
    main()
