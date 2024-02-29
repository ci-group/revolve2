"""Manually control a physical robot to test if it works as expected."""
from revolve2.modular_robot.body import RightAngles
from revolve2.modular_robot.body.base import ActiveHinge
from revolve2.modular_robot.body.sensors import ActiveHingeSensor
from revolve2.modular_robot.body.v2 import ActiveHingeV2, BodyV2, BrickV2
from revolve2.modular_robot_physical import UUIDKey
from revolve2.modular_robot_physical.remote import test_physical_robot


def make_body() -> (
    tuple[BodyV2, tuple[ActiveHinge, ActiveHinge, ActiveHinge, ActiveHinge]]
):
    """
    Create a body for the robot.

    :returns: The created body and a tuple of all ActiveHinge objects for mapping later on.
    """
    # A modular robot body follows a 'tree' structure.
    # The 'Body' class automatically creates a center 'core'.
    # From here, other modular can be attached.
    # Modules can be attached in a rotated fashion.
    # This can be any angle, although the original design takes into account only multiples of 90 degrees.
    body = BodyV2()
    body.core_v2.left_face.bottom = ActiveHingeV2(RightAngles.DEG_0)
    body.core_v2.left_face.bottom.add_sensor(ActiveHingeSensor())
    body.core_v2.left_face.bottom.attachment = ActiveHingeV2(RightAngles.DEG_0)
    body.core_v2.left_face.bottom.attachment.add_sensor(ActiveHingeSensor())
    body.core_v2.left_face.bottom.attachment.attachment = BrickV2(RightAngles.DEG_0)

    body.core_v2.right_face.bottom = ActiveHingeV2(RightAngles.DEG_0)
    body.core_v2.right_face.bottom.add_sensor(ActiveHingeSensor())
    body.core_v2.right_face.bottom.attachment = ActiveHingeV2(RightAngles.DEG_0)
    body.core_v2.right_face.bottom.attachment.add_sensor(ActiveHingeSensor())
    body.core_v2.right_face.bottom.attachment.attachment = BrickV2(RightAngles.DEG_0)

    """Here we collect all ActiveHinges, to map them later onto the physical robot."""
    active_hinges = (
        body.core_v2.left_face.bottom,
        body.core_v2.left_face.bottom.attachment,
        body.core_v2.right_face.bottom,
        body.core_v2.right_face.bottom.attachment,
    )
    return body, active_hinges


def main() -> None:
    """Run the main entry of the script."""
    body, hinges = make_body()
    hinge_1, hinge_2, hinge_3, hinge_4 = hinges
    hinge_mapping = {
        UUIDKey(hinge_1): 0,
        UUIDKey(hinge_2): 15,
        UUIDKey(hinge_3): 16,
        UUIDKey(hinge_4): 31,
    }
    test_physical_robot(
        robot=body,
        hostname="localhost",  # "Set the robot IP here.
        inverse_servos={},
        hinge_mapping=hinge_mapping,
    )


if __name__ == "__main__":
    main()
