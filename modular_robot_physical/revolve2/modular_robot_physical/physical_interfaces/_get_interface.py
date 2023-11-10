from .._hardware_type import HardwareType
from ._physical_interface import PhysicalInterface


def get_interface(
    hardware_type: HardwareType, debug: bool, dry: bool, pins: list[int], careful: bool
) -> PhysicalInterface:
    """
    Get the interface for the given hardware type.

    :param hardware_type: The type of hardware.
    :param debug: If debugging messages are activated.
    :param dry: If servo outputs are not propagated to the physical servos.:
    :param pins: The GPIO pins that will be used.
    :param careful: Enable careful mode, which slowly steps the servo to its target, instead of going as fast as possible. This decreases current drawn by the motors, which might be necessary for some robots. This is only available for V2 robots.
    :returns: The interface.
    :raises NotImplementedError: If the hardware type is not supported or if careful is enable and not supported for the hardware type.
    :raises ModuleNotFoundError: If some required package are not installed.
    """
    try:
        match hardware_type:
            case HardwareType.v1:
                from ..physical_interfaces.v1 import V1PhysicalInterface

                if careful:
                    raise NotImplementedError(
                        "Careful mode is not available for V1 robots."
                    )

                return V1PhysicalInterface(
                    debug=debug,
                    dry=dry,
                    pins=pins,
                )
            case HardwareType.v2:
                from ..physical_interfaces.v2 import V2PhysicalInterface

                return V2PhysicalInterface(
                    debug=debug, dry=dry, pins=pins, careful=careful
                )
            case _:
                raise NotImplementedError()
    except ModuleNotFoundError as e:
        raise ModuleNotFoundError(
            f"Could not import physical interface, did you install the required extras? Error: {e}"
        ) from None
