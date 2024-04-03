from revolve2.modular_robot.body.base import ActiveHinge, AttachmentFace, Brick, Core
from revolve2.modular_robot.body.sensors import (
    ActiveHingeSensor,
    CameraSensor,
    IMUSensor,
)

from ._builders import (
    ActiveHingeBuilder,
    ActiveHingeSensorBuilder,
    AttachmentFaceBuilder,
    BrickBuilder,
    Builder,
    CameraSensorBuilder,
    CoreBuilder,
    IMUSensorBuilder,
)
from ._unbuilt_child import UnbuiltChild


def get_builder(unbuilt_child: UnbuiltChild) -> Builder:
    """
    Get and initialize the corresponding Builder for the module.

    :param unbuilt_child: The target child to be built.
    :return: The initialized Builder.
    :raises KeyError: If no Builder available for the Module type.
    """
    match unbuilt_child.child_object:
        case Core():
            return CoreBuilder(
                module=unbuilt_child.child_object,
                rigid_body=unbuilt_child.rigid_body,
                slot_pose=unbuilt_child.pose,
            )
        case Brick():
            return BrickBuilder(
                module=unbuilt_child.child_object,
                rigid_body=unbuilt_child.rigid_body,
                slot_pose=unbuilt_child.pose,
            )
        case ActiveHinge():
            return ActiveHingeBuilder(
                module=unbuilt_child.child_object,
                rigid_body=unbuilt_child.rigid_body,
                slot_pose=unbuilt_child.pose,
            )
        case AttachmentFace():
            return AttachmentFaceBuilder(
                module=unbuilt_child.child_object,
                rigid_body=unbuilt_child.rigid_body,
                slot_pose=unbuilt_child.pose,
            )
        case ActiveHingeSensor():
            return ActiveHingeSensorBuilder(
                sensor=unbuilt_child.child_object,
                rigid_body=unbuilt_child.rigid_body,
            )
        case IMUSensor():
            return IMUSensorBuilder(
                sensor=unbuilt_child.child_object,
                rigid_body=unbuilt_child.rigid_body,
                pose=unbuilt_child.pose,
                imu_location=unbuilt_child.child_object.position,
            )
        case CameraSensor():
            return CameraSensorBuilder(
                sensor=unbuilt_child.child_object,
                rigid_body=unbuilt_child.rigid_body,
                pose=unbuilt_child.pose,
            )
        case _:
            raise KeyError(
                f"Module of type {type(unbuilt_child.child_object)} has no defined Builder."
            )
