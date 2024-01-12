"""Manually control a physical robot to test if it works as expected."""
from revolve2.modular_robot.body import RightAngles
from revolve2.modular_robot.body.base import ActiveHinge, ActiveHingeSensor
from revolve2.modular_robot.body.v1 import ActiveHingeV1, BodyV1, BrickV1
from revolve2.modular_robot_physical import UUIDKey
from revolve2.modular_robot_physical.remote import test_physical_robot


def make_body() -> (
    tuple[BodyV1, tuple[ActiveHinge, ActiveHinge, ActiveHinge, ActiveHinge]]
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
    body = BodyV1()
    body.core_v1.left = ActiveHingeV1(RightAngles.DEG_0)
    body.core_v1.left.sensor = ActiveHingeSensor()
    body.core_v1.left.attachment = ActiveHingeV1(RightAngles.DEG_0)
    body.core_v1.left.attachment.sensor = ActiveHingeSensor()
    body.core_v1.left.attachment.attachment = BrickV1(RightAngles.DEG_0)
    body.core_v1.right = ActiveHingeV1(RightAngles.DEG_0)
    body.core_v1.right.sensor = ActiveHingeSensor()
    body.core_v1.right.attachment = ActiveHingeV1(RightAngles.DEG_0)
    body.core_v1.right.attachment.sensor = ActiveHingeSensor()
    body.core_v1.right.attachment.attachment = BrickV1(RightAngles.DEG_0)

    """Here we collect all ActiveHinges, to map them later onto the physical robot."""
    active_hinges = (
        body.core_v1.left,
        body.core_v1.left.attachment,
        body.core_v1.right,
        body.core_v1.right.attachment,
    )
    return body, active_hinges


def main() -> None:
    """Main entry of the script."""
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
