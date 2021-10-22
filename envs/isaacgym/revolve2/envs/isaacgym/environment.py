from typing import Optional

from revolve2.core.physics_robot import PhysicsRobot


class Environment:
    robot: Optional[PhysicsRobot]

    def __init__(self):
        self.robot = None
