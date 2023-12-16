from revolve2.modular_robot.body.base import ActiveHinge, AttachmentFace, Brick, Core

from ._active_hinge_builder import ActiveHingeBuilder
from ._attachment_face_builder import AttachmentFaceBuilder
from ._brick_builder import BrickBuilder
from ._builder import Builder
from ._core_builder import CoreBuilder
from ._unbuilt_child import UnbuiltChild


def get_builder(unbuilt_child: UnbuiltChild) -> Builder:
    """
    Get and initialize the corresponding Builder for the module.

    :param unbuilt_child: The target child to be built.
    :return: The initialized Builder.
    :raises KeyError: If no Builder available for the Module type.
    """
    match unbuilt_child.module:
        case Core():
            return CoreBuilder(
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
        case AttachmentFace():
            return AttachmentFaceBuilder(
                module=unbuilt_child.module,
                rigid_body=unbuilt_child.rigid_body,
                slot_pose=unbuilt_child.pose,
            )
        case _:
            raise KeyError(
                f"Module of type {type(unbuilt_child.module)} has no defined Builder."
            )
