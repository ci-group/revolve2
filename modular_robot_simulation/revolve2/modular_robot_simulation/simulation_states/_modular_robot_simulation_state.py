from revolve2.modular_robot.body import Module
from revolve2.simulation.scene import Pose
from ._multi_body_system_simulation_state import MultiBodySystemSimulationState


class ModularRobotSimulationState(MultiBodySystemSimulationState):
    """The state of a modular robot at some moment in a simulation."""

    def get_module_relative_pose(self, module: Module) -> Pose:
        """
        Get the pose of a module, relative to its parent module's reference frame.

        In case there is no parent(the core), this is equal to getting the absolute pose.

        :param module: The module to get the pose for.
        :returns: The relative pose.
        :raises NotImplementedError: Always.
        """
        raise NotImplementedError()
        return Pose()

    def get_module_absolute_pose(self, module: Module) -> Pose:
        """
        Get the pose of this module, relative the global reference frame.

        :param module: The module to get the pose for.
        :returns: The absolute pose.
        :raises NotImplementedError: Always.
        """
        raise NotImplementedError()
        return Pose()
