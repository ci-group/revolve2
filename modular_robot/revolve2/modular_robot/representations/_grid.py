from revolve2.modular_robot import Directions, RightAngles
from typing import Optional


class Grid:
    def __init__(self):
        self.min_x: Optional[int] = None
        self.max_x: Optional[int] = None
        self.min_y: Optional[int] = None
        self.max_y: Optional[int] = None
        self.width: Optional[int] = None
        self.height: Optional[int] = None
        self.core_position = None
        self.visited_coordinates = []

        self.x_pos = 0
        self.y_pos = 0
        self.orientation = Directions.FRONT
        self.previous_move: Optional[Directions] = None
        self.movement_stack = [[0, 0, Directions.FRONT]]

    def get_position(self):
        """Return current position on x and y axis"""
        return [self.x_pos, self.y_pos]

    def set_position(self, x, y):
        """Set position of x and y axis"""
        self.x_pos = x
        self.y_pos = y

    def set_orientation(self, orientation: Directions):
        """Set new orientation on grid"""
        if Directions.has(orientation):
            self.orientation = orientation
        else:
            return False

    def calculate_orientation(self):
        """Set orientation by previous move and orientation"""
        if self.previous_move is not None:
            self.set_orientation(
                Directions.from_angle(
                    self.previous_move.to_angle() + self.orientation.to_angle()
                )
            )
        else:
            raise ValueError("Cannot calculate orientation: invalid state")

    def move_by_slot(self, slot: Directions | int):
        """Move in direction by slot id"""
        if not isinstance(slot, Directions):
            slot = Directions(slot)

        match self.orientation.to_angle() + slot.to_angle():
            case RightAngles.RAD_0:
                self.y_pos -= 1
            case RightAngles.RAD_HALFPI:
                self.x_pos += 1
            case RightAngles.RAD_PI:
                self.y_pos += 1
            case RightAngles.RAD_ONEANDAHALFPI:
                self.x_pos -= 1

        self.previous_move = slot

    def move_back(self):
        if len(self.movement_stack) > 1:
            self.movement_stack.pop()
        last_movement = self.movement_stack[-1]
        self.x_pos = last_movement[0]
        self.y_pos = last_movement[1]
        self.orientation = last_movement[2]

    def add_to_visited(self, include_sensors=True, is_sensor=False):
        """Add current position to visited coordinates list"""
        self.calculate_orientation()
        if (include_sensors and is_sensor) or not is_sensor:
            self.visited_coordinates.append([self.x_pos, self.y_pos])
        self.movement_stack.append([self.x_pos, self.y_pos, self.orientation])

    def calculate_grid_dimensions(self):
        min_x = 0
        max_x = 0
        min_y = 0
        max_y = 0
        for coorinate in self.visited_coordinates:
            min_x = coorinate[0] if coorinate[0] < min_x else min_x
            max_x = coorinate[0] if coorinate[0] > max_x else max_x
            min_y = coorinate[1] if coorinate[1] < min_y else min_y
            max_y = coorinate[1] if coorinate[1] > max_y else max_y

        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.width = abs(min_x - max_x) + 1
        self.height = abs(min_y - max_y) + 1

    def calculate_core_position(self):
        if (
            self.width is not None
            and self.max_x is not None
            and self.height is not None
            and self.max_y is not None
        ):
            self.core_position = [
                self.width - self.max_x - 1,
                self.height - self.max_y - 1,
            ]
            return self.core_position

        raise ValueError("Cannot calculate core position, position not set")

    def reset_grid(self):
        self.x_pos = 0
        self.y_pos = 0
        self.orientation = Directions.FRONT
        self.previous_move = None
        self.movement_stack = [[0, 0, Directions.FRONT]]
