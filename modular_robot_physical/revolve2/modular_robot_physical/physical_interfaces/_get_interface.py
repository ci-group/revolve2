from ._harware_type import HardwareType
from ._physical_interface import PhysicalInterface


def get_interface(
    hardware_type: HardwareType, debug: bool, dry: bool, pins: list[int]
) -> PhysicalInterface:
    """
    Get the interface for the given hardware type.

    :param hardware_type: The type of hardware.
    :param debug: If debugging messages are activated.
    :param dry: If servo outputs are not propagated to the physical servos.:
    :param pins: The GPIO pins that will be used.
    :returns: The interface.
    :raises NotImplementedError: If the hardware type is not supported.
    """
    match hardware_type:
        case HardwareType.v1:
            from ..physical_interfaces.v1 import V1PhysicalInterface

            return V1PhysicalInterface(
                debug=debug,
                dry=dry,
                pins=pins,
            )
        case _:
            raise NotImplementedError()
