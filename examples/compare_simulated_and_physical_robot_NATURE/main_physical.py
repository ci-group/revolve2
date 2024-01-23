"""Manually control a physical robot to test if it works as expected."""
from revolve2.modular_robot.body import RightAngles
from revolve2.modular_robot.body.base import ActiveHinge, ActiveHingeSensor
from revolve2.modular_robot.body.v1 import ActiveHingeV1, BodyV1, BrickV1
from revolve2.modular_robot_physical import UUIDKey
from revolve2.modular_robot_physical.remote import test_physical_robot


def make_body() -> (
    BodyV1
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
    body.core_v1.front = ActiveHingeV1(RightAngles.DEG_90)
    body.core_v1.front.attachment = ActiveHingeV1(RightAngles.DEG_270)
    body.core_v1.front.attachment.attachment = ActiveHingeV1(RightAngles.DEG_0)

    body.core_v1.right = ActiveHingeV1(RightAngles.DEG_90)
    body.core_v1.right.attachment = ActiveHingeV1(RightAngles.DEG_270)
    body.core_v1.right.attachment.attachment = ActiveHingeV1(RightAngles.DEG_0)

    body.core_v1.back = ActiveHingeV1(RightAngles.DEG_90)
    body.core_v1.back.attachment = ActiveHingeV1(RightAngles.DEG_270)
    body.core_v1.back.attachment.attachment = ActiveHingeV1(RightAngles.DEG_0)


    return body


def main() -> None:
    """Run the main entry of the script."""
    body = make_body()
    hinges = body.find_modules_of_type(ActiveHinge)
    pins = [17, 27, 22, 23, 18, 24, 25, 9, 10]

    hinge_mapping = {UUIDKey(hinge): pin for hinge, pin in zip(hinges, pins)}
    test_physical_robot(
        robot=body,
        hostname="10.15.3.98",  # "Set the robot IP here.
        inverse_servos={},
        hinge_mapping=hinge_mapping,
    )


if __name__ == "__main__":
    main()
