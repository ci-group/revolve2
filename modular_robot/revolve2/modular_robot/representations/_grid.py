from revolve2.modular_robot import Directions
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

    # Current position of last drawn element
    x_pos = 0
    y_pos = 0

    # Orientation of robot

    orientation = Directions.FRONT

    # Direction of last movement
    previous_move: Optional[Directions] = None

    # Coordinates and orientation of movements
    movement_stack = [[0, 0, Directions.FRONT]]

    def get_position(self):
        """Return current position on x and y axis"""
        return [Grid.x_pos, Grid.y_pos]

    def set_position(self, x, y):
        """Set position of x and y axis"""
        Grid.x_pos = x
        Grid.y_pos = y

    def set_orientation(self, orientation):
        """Set new orientation on grid"""
        if Directions.has(orientation):
            Grid.orientation = orientation
        else:
            return False

    def calculate_orientation(self):
        """Set orientation by previous move and orientation"""
        if Grid.previous_move is not None:
            self.set_orientation(
                Directions.from_angle(
                    Grid.previous_move.to_angle() + Grid.orientation.to_angle()
                )
            )
        else:
            raise ValueError("Cannot calculate orientation: invalid state")

    def move_by_slot(self, slot):
        """Move in direction by slot id"""
        if slot == Directions.BACK:
            self.move_down()
        elif slot == Directions.FRONT:
            self.move_up()
        elif slot == Directions.RIGHT:
            self.move_right()
        elif slot == Directions.LEFT:
            self.move_left()

    def move_right(self):
        """Set position one to the right in correct orientation"""
        if Grid.orientation == Directions.FRONT:
            Grid.x_pos += 1
        elif Grid.orientation == Directions.RIGHT:
            Grid.y_pos += 1
        elif Grid.orientation == Directions.BACK:
            Grid.x_pos -= 1
        elif Grid.orientation == Directions.LEFT:
            Grid.y_pos -= 1
        Grid.previous_move = Directions.RIGHT

    def move_left(self):
        """Set position one to the left"""
        if Grid.orientation == Directions.FRONT:
            Grid.x_pos -= 1
        elif Grid.orientation == Directions.RIGHT:
            Grid.y_pos -= 1
        elif Grid.orientation == Directions.BACK:
            Grid.x_pos += 1
        elif Grid.orientation == Directions.LEFT:
            Grid.y_pos += 1
        Grid.previous_move = Directions.LEFT

    def move_up(self):
        """Set position one upwards"""
        if Grid.orientation == Directions.FRONT:
            Grid.y_pos -= 1
        elif Grid.orientation == Directions.RIGHT:
            Grid.x_pos += 1
        elif Grid.orientation == Directions.BACK:
            Grid.y_pos += 1
        elif Grid.orientation == Directions.LEFT:
            Grid.x_pos -= 1
        Grid.previous_move = Directions.FRONT

    def move_down(self):
        """Set position one downwards"""
        if Grid.orientation == Directions.FRONT:
            Grid.y_pos += 1
        elif Grid.orientation == Directions.RIGHT:
            Grid.x_pos -= 1
        elif Grid.orientation == Directions.BACK:
            Grid.y_pos -= 1
        elif Grid.orientation == Directions.LEFT:
            Grid.x_pos += 1
        Grid.previous_move = Directions.BACK

    def move_back(self):
        if len(Grid.movement_stack) > 1:
            Grid.movement_stack.pop()
        last_movement = Grid.movement_stack[-1]
        Grid.x_pos = last_movement[0]
        Grid.y_pos = last_movement[1]
        Grid.orientation = last_movement[2]

    def add_to_visited(self, include_sensors=True, is_sensor=False):
        """Add current position to visited coordinates list"""
        self.calculate_orientation()
        if (include_sensors and is_sensor) or not is_sensor:
            self.visited_coordinates.append([Grid.x_pos, Grid.y_pos])
        Grid.movement_stack.append([Grid.x_pos, Grid.y_pos, Grid.orientation])

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
        Grid.x_pos = 0
        Grid.y_pos = 0
        Grid.orientation = Directions.FRONT
        Grid.previous_move = None
        Grid.movement_stack = [[0, 0, Directions.FRONT]]
