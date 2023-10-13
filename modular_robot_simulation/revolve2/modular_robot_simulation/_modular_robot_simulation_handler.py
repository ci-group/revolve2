from revolve2.modular_robot import ModularRobotControlInterface, ModularRobotSensorState
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.brain import BrainInstance
from revolve2.simulation.scene import (
    ControlInterface,
    JointHinge,
    SimulationHandler,
    SimulationState,
)

from ._uuid_key import UUIDKey


class _ModularRobotControlInterfaceImpl(ModularRobotControlInterface):
    _simulation_control: ControlInterface
    _joint_mapping: dict[UUIDKey[ActiveHinge], JointHinge]

    def __init__(
        self,
        simulation_control: ControlInterface,
        joint_mapping: dict[UUIDKey[ActiveHinge], JointHinge],
    ) -> None:
        self._simulation_control = simulation_control
        self._joint_mapping = joint_mapping

    def set_active_hinge_target(self, active_hinge: ActiveHinge, target: float) -> None:
        self._simulation_control.set_joint_hinge_position_target(
            self._joint_mapping[UUIDKey(active_hinge)], target
        )


class ModularRobotSimulationHandler(SimulationHandler):
    """Implements the simulation handler for a modular robot scene."""

    _brains: list[
        tuple[
            BrainInstance,
            dict[UUIDKey[ActiveHinge], JointHinge],
        ]
    ]

    def __init__(self) -> None:
        """Initialize this object."""
        self._brains = []

    def add_robot(
        self,
        brain_instance: BrainInstance,
        joint_mapping: dict[UUIDKey[ActiveHinge], JointHinge],
    ) -> None:
        """
        Add a brain that will control a robot during simulation.

        :param brain_instance: The brain.
        :param joint_mapping: A mapping from modular robot active hinges to simulation hinge joints.
        """
        self._brains.append((brain_instance, joint_mapping))

    def handle(
        self,
        simulation_state: SimulationState,
        simulation_control: ControlInterface,
        dt: float,
    ) -> None:
        """
        Handle a simulation frame.

        :param simulation_state: The current state of the simulation.
        :param simulation_control: Interface for setting control targets.
        :param dt: The time since the last call to this function.
        """
        for (
            brain_instance,
            joint_mapping,
        ) in self._brains:
            sensor_state = ModularRobotSensorState()
            control = _ModularRobotControlInterfaceImpl(
                simulation_control=simulation_control, joint_mapping=joint_mapping
            )
            brain_instance.control(
                dt=dt, sensor_state=sensor_state, control_interface=control
            )
