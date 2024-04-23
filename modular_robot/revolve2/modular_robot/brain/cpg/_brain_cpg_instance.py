import numpy as np
import numpy.typing as npt

from ..._modular_robot_control_interface import ModularRobotControlInterface
from ...body.base import ActiveHinge
from ...sensor_state import ModularRobotSensorState
from .._brain_instance import BrainInstance


class BrainCpgInstance(BrainInstance):
    """
    CPG network brain.

    A state array that is integrated over time following the differential equation `X'=WX`.
    W is a weight matrix that is multiplied by the state array.
    The outputs of the controller are defined by the `outputs`, a list of indices for the state array.
    """

    _initial_state: npt.NDArray[np.float_]
    _weight_matrix: npt.NDArray[np.float_]  # nxn matrix matching number of neurons
    _output_mapping: list[tuple[int, ActiveHinge]]

    def __init__(
        self,
        initial_state: npt.NDArray[np.float_],
        weight_matrix: npt.NDArray[np.float_],
        output_mapping: list[tuple[int, ActiveHinge]],
    ) -> None:
        """
        Initialize this CPG Brain Instance.

        :param initial_state: The initial state of the neural network.
        :param weight_matrix: The weight matrix used during integration.
        :param output_mapping: Marks neurons as controller outputs and map them to the correct active hinge.
        """
        assert initial_state.ndim == 1
        assert weight_matrix.ndim == 2
        assert weight_matrix.shape[0] == weight_matrix.shape[1]
        assert initial_state.shape[0] == weight_matrix.shape[0]
        assert all([i >= 0 and i < len(initial_state) for i, _ in output_mapping])

        self._state = initial_state
        self._weight_matrix = weight_matrix
        self._output_mapping = output_mapping

    @staticmethod
    def _rk45(
        state: npt.NDArray[np.float_], A: npt.NDArray[np.float_], dt: float
    ) -> npt.NDArray[np.float_]:
        """
        Calculate the next state using the RK45 method.

        This implementation of the Runge–Kutta–Fehlberg method allows us to improve accuracy of state calculations by comparing solutions at different step sizes.
        For more info see: See https://en.wikipedia.org/wiki/Runge%E2%80%93Kutta%E2%80%93Fehlberg_method.
        RK45 is a method of order 4 with an error estimator of order 5 (Fehlberg, E. (1969). Low-order classical Runge-Kutta formulas with stepsize control. NASA Technical Report R-315.).

        :param state: The current state of the network.
        :param A: The weights matrix of the network.
        :param dt: The step size (elapsed simulation time).
        :return: The new state.
        """
        A1: npt.NDArray[np.float_] = np.matmul(A, state)
        A2: npt.NDArray[np.float_] = np.matmul(A, (state + dt / 2 * A1))
        A3: npt.NDArray[np.float_] = np.matmul(A, (state + dt / 2 * A2))
        A4: npt.NDArray[np.float_] = np.matmul(A, (state + dt * A3))
        state = state + dt / 6 * (A1 + 2 * (A2 + A3) + A4)
        return np.clip(state, a_min=-1, a_max=1)

    def control(
        self,
        dt: float,
        sensor_state: ModularRobotSensorState,
        control_interface: ModularRobotControlInterface,
    ) -> None:
        """
        Control the modular robot.

        Sets the active hinge targets to the values in the state array as defined by the mapping provided in the constructor.

        :param dt: Elapsed seconds since last call to this function.
        :param sensor_state: Interface for reading the current sensor state.
        :param control_interface: Interface for controlling the robot.
        """
        # Integrate ODE to obtain new state.
        self._state = self._rk45(self._state, self._weight_matrix, dt)

        # Set active hinge targets to match newly calculated state.
        for state_index, active_hinge in self._output_mapping:
            control_interface.set_active_hinge_target(
                active_hinge, float(self._state[state_index]) * active_hinge.range
            )
