from revolve2.modular_robot.body.base import ActiveHinge, Brick
from revolve2.modular_robot.body.v1 import CoreV1
from revolve2.modular_robot.body.v2 import CoreV2

from ._active_hinge_builder import ActiveHingeBuilder
from ._brick_builder import BrickBuilder
from ._builder import Builder
from ._core_v1_builder import CoreV1Builder
from ._core_v2_builder import CoreV2Builder
from ._unbuilt_child import UnbuiltChild


def get_builder(unbuilt_child: UnbuiltChild) -> Builder:
    """
    Get and initialize the corresponding Builder for the module.

    :param unbuilt_child: The target child to be built.
    :return: The initialized Builder.
    :raises KeyError: If no Builder available for the Module type.
    """
    match unbuilt_child.module:
        case CoreV1():
            return CoreV1Builder(
                module=unbuilt_child.module,
                rigid_body=unbuilt_child.rigid_body,
                slot_pose=unbuilt_child.pose,
            )
        case CoreV2():
            return CoreV2Builder(
                module=unbuilt_child.module,
                rigid_body=unbuilt_child.rigid_body,
                slot_pose=unbuilt_child.pose,
            )
        case Brick():
            return BrickBuilder(
                module=unbuilt_child.module,
                rigid_body=unbuilt_child.rigid_body,
                slot_pose=unbuilt_child.pose,
            )
        case ActiveHinge():
            return ActiveHingeBuilder(
                module=unbuilt_child.module,
                rigid_body=unbuilt_child.rigid_body,
                slot_pose=unbuilt_child.pose,
            )
        case _:
            raise KeyError(
                f"Module of type {type(unbuilt_child.module)} has no defined Builder."
            )
