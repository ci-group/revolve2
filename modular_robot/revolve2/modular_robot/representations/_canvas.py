import cairo
import math

from revolve2.modular_robot import Directions, RightAngles
from typing import Optional


class Canvas:
    # Current position of last drawn element
    x_pos = 0
    y_pos = 0

    # Orientation of robot
    orientation = Directions.FRONT

    # Direction of last movement
    previous_move: Optional[Directions] = None

    # Coordinates and orientation of movements
    movement_stack = []

    # Positions for the sensors
    sensors = []

    # Rotating orientation in regard to parent module
    rotating_orientation = 0

    def __init__(self, width, height, scale):
        """Instantiate context and surface"""
        self.surface = cairo.ImageSurface(
            cairo.FORMAT_ARGB32, width * scale, height * scale
        )
        context = cairo.Context(self.surface)
        context.scale(scale, scale)
        self.context = context
        self.width = width
        self.height = height
        self.scale = scale

    def get_position(self):
        """Return current position on x and y axis"""
        return [Canvas.x_pos, Canvas.y_pos]

    def set_position(self, x, y):
        """Set position of x and y axis"""
        Canvas.x_pos = x
        Canvas.y_pos = y

    def set_orientation(self, orientation: Directions):
        """Set new orientation of robot"""
        if Directions.has(orientation):
            Canvas.orientation = orientation
        else:
            raise TypeError(f"{orientation} is not one of {list(Directions)}")

    def calculate_orientation(self):
        """Calculate new orientation based on current orientation and last movement direction"""

        if Canvas.previous_move is None:
            self.set_orientation(Directions.FRONT)
        else:
            self.set_orientation(
                Directions.from_angle(
                    Canvas.previous_move.to_angle() + Canvas.orientation.to_angle()
                )
            )

        return

    def move_by_slot(self, slot: Directions | int):
        """Move in direction by slot id"""
        if not isinstance(slot, Directions):
            slot = Directions(slot)

        match Canvas.orientation.to_angle() + slot.to_angle():
            case RightAngles.RAD_0:
                Canvas.y_pos -= 1
            case RightAngles.RAD_HALFPI:
                Canvas.x_pos += 1
            case RightAngles.RAD_PI:
                Canvas.y_pos += 1
            case RightAngles.RAD_ONEANDAHALFPI:
                Canvas.x_pos -= 1

        Canvas.previous_move = slot

    def move_back(self):
        """Move Directions.BACK to previous state on canvas"""
        if len(Canvas.movement_stack) > 1:
            Canvas.movement_stack.pop()
        last_movement = Canvas.movement_stack[-1]
        Canvas.x_pos = last_movement[0]
        Canvas.y_pos = last_movement[1]
        Canvas.orientation = last_movement[2]
        Canvas.rotating_orientation = last_movement[3]

    def sign_id(self, mod_id):
        """Sign module with the id on the upper Directions.LEFT corner of block"""
        self.context.set_font_size(0.3)
        self.context.move_to(Canvas.x_pos, Canvas.y_pos + 0.4)
        self.context.set_source_rgb(0, 0, 0)
        if type(mod_id) is int:
            self.context.show_text(str(mod_id))
        else:
            mod_id = "".join(x for x in mod_id if x.isdigit())
            self.context.show_text(mod_id)
        self.context.stroke()

    def draw_controller(self, mod_id):
        """Draw a controller (yellow) in the middle of the canvas"""
        self.context.rectangle(Canvas.x_pos, Canvas.y_pos, 1, 1)
        self.context.set_source_rgb(255, 255, 0)
        self.context.fill_preserve()
        self.context.set_source_rgb(0, 0, 0)
        self.context.set_line_width(0.01)
        self.context.stroke()
        self.sign_id(mod_id)
        Canvas.movement_stack.append(
            [
                Canvas.x_pos,
                Canvas.y_pos,
                Canvas.orientation,
                Canvas.rotating_orientation,
            ]
        )

    def draw_hinge(self, mod_id):
        """Draw a hinge (blue) on the previous object"""

        self.context.rectangle(Canvas.x_pos, Canvas.y_pos, 1, 1)
        if Canvas.rotating_orientation == 0:
            self.context.set_source_rgb(1.0, 0.4, 0.4)
        else:
            self.context.set_source_rgb(1, 0, 0)
        self.context.fill_preserve()
        self.context.set_source_rgb(0, 0, 0)
        self.context.set_line_width(0.01)
        self.context.stroke()
        self.calculate_orientation()
        self.sign_id(mod_id)
        Canvas.movement_stack.append(
            [
                Canvas.x_pos,
                Canvas.y_pos,
                Canvas.orientation,
                Canvas.rotating_orientation,
            ]
        )

    def draw_module(self, mod_id):
        """Draw a module (red) on the previous object"""
        self.context.rectangle(Canvas.x_pos, Canvas.y_pos, 1, 1)
        self.context.set_source_rgb(0, 0, 1)
        self.context.fill_preserve()
        self.context.set_source_rgb(0, 0, 0)
        self.context.set_line_width(0.01)
        self.context.stroke()
        self.calculate_orientation()
        self.sign_id(mod_id)
        Canvas.movement_stack.append(
            [
                Canvas.x_pos,
                Canvas.y_pos,
                Canvas.orientation,
                Canvas.rotating_orientation,
            ]
        )

    def calculate_sensor_rectangle_position(self):
        """Calculate squeezed sensor rectangle position based on current orientation and last movement direction"""
        if Canvas.previous_move is None:
            return Canvas.x_pos, Canvas.y_pos + 0.9, 1, 0.1
        else:
            match Canvas.previous_move.to_angle() + Canvas.orientation.to_angle():
                case RightAngles.RAD_0:
                    return Canvas.x_pos, Canvas.y_pos + 0.9, 1, 0.1
                case RightAngles.RAD_HALFPI:
                    return Canvas.x_pos, Canvas.y_pos, 0.1, 1
                case RightAngles.RAD_PI:
                    return Canvas.x_pos, Canvas.y_pos, 1, 0.1
                case RightAngles.RAD_ONEANDAHALFPI:
                    return Canvas.x_pos + 0.9, Canvas.y_pos, 0.1, 1

        raise ValueError("Cannot calculate sensor position: Invalid")

    def save_sensor_position(self):
        """Save sensor position in list"""
        x, y, x_scale, y_scale = self.calculate_sensor_rectangle_position()
        Canvas.sensors.append([x, y, x_scale, y_scale])
        self.calculate_orientation()
        Canvas.movement_stack.append(
            [
                Canvas.x_pos,
                Canvas.y_pos,
                Canvas.orientation,
                Canvas.rotating_orientation,
            ]
        )

    def draw_sensors(self):
        """Draw all sensors"""
        for sensor in Canvas.sensors:
            self.context.rectangle(sensor[0], sensor[1], sensor[2], sensor[3])
            self.context.set_source_rgb(0.6, 0.6, 0.6)
            self.context.fill_preserve()
            self.context.set_source_rgb(0, 0, 0)
            self.context.set_line_width(0.01)
            self.context.stroke()

    def calculate_connector_to_parent_position(self):
        """Calculate position of connector node on canvas"""
        parent = Canvas.movement_stack[-2]
        parent_orientation = parent[2]

        if Canvas.previous_move is not None:
            match Canvas.previous_move.to_angle() + parent_orientation.to_angle():
                case RightAngles.RAD_0:
                    return parent[0] + 0.5, parent[1]
                case RightAngles.RAD_HALFPI:
                    return parent[0] + 1, parent[1] + 0.5
                case RightAngles.RAD_PI:
                    return parent[0] + 0.5, parent[1] + 1
                case RightAngles.RAD_ONEANDAHALFPI:
                    return parent[0], parent[1] + 0.5

        raise ValueError("Invalid position to calculate parent from")

    def draw_connector_to_parent(self):
        """Draw a circle between child and parent"""
        x, y = self.calculate_connector_to_parent_position()
        self.context.arc(x, y, 0.1, 0, math.pi * 2)
        self.context.set_source_rgb(0, 0, 0)
        self.context.fill_preserve()
        self.context.set_source_rgb(0, 0, 0)
        self.context.set_line_width(0.01)
        self.context.stroke()

    def save_png(self, file_name):
        """Store image representation of canvas"""
        self.surface.write_to_png(str(file_name))

    def reset_canvas(self):
        """Reset canvas variables to default values"""
        Canvas.x_pos = 0
        Canvas.y_pos = 0
        Canvas.orientation = Directions.FRONT
        Canvas.previous_move = None
        Canvas.movement_stack = []
        Canvas.sensors = []
        Canvas.rotating_orientation = 0
