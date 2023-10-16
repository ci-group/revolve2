import cairo
import math
from pathlib import Path

from revolve2.modular_robot import Directions, RightAngles
from typing import Optional, List


class Canvas:
    def __init__(self, width: int, height: int, scale: int):
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

        self.x_pos = 0
        self.y_pos = 0
        self.orientation = Directions.FRONT
        self.previous_move: Optional[Directions] = None
        self.movement_stack = []
        self.sensors = []
        self.rotating_orientation = 0

    def get_position(self) -> List[int]:
        """Return current position on x and y axis"""
        return [self.x_pos, self.y_pos]

    def set_position(self, x: int, y: int) -> None:
        """Set position of x and y axis"""
        self.x_pos = x
        self.y_pos = y

    def set_orientation(self, orientation: Directions) -> None:
        """Set new orientation of robot"""
        if Directions.has(orientation):
            self.orientation = orientation
        else:
            raise TypeError(f"{orientation} is not one of {list(Directions)}")

    def calculate_orientation(self) -> None:
        """Calculate new orientation based on current orientation and last movement direction"""

        if self.previous_move is None:
            self.set_orientation(Directions.FRONT)
        else:
            self.set_orientation(
                Directions.from_angle(
                    self.previous_move.to_angle() + self.orientation.to_angle()
                )
            )

        return

    def move_by_slot(self, slot: Directions) -> None:
        """Move in direction by slot id"""

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
        """Move Directions.BACK to previous state on canvas"""
        if len(self.movement_stack) > 1:
            self.movement_stack.pop()
        last_movement = self.movement_stack[-1]
        self.x_pos = last_movement[0]
        self.y_pos = last_movement[1]
        self.orientation = last_movement[2]
        self.rotating_orientation = last_movement[3]

    def sign_id(self, mod_id: int | str):
        """Sign module with the id on the upper Directions.LEFT corner of block"""
        self.context.set_font_size(0.3)
        self.context.move_to(self.x_pos, self.y_pos + 0.4)
        self.context.set_source_rgb(0, 0, 0)
        if isinstance(mod_id, int):
            self.context.show_text(str(mod_id))
        elif isinstance(mod_id, str):
            mod_id = "".join(x for x in mod_id if x.isdigit())
            self.context.show_text(mod_id)
        else:
            raise TypeError(f"Type of mod_id: {mod_id} is not one of: string, int")
        self.context.stroke()

    def draw_controller(self, mod_id: int | str):
        """Draw a controller (yellow) in the middle of the canvas"""
        self.context.rectangle(self.x_pos, self.y_pos, 1, 1)
        self.context.set_source_rgb(255, 255, 0)
        self.context.fill_preserve()
        self.context.set_source_rgb(0, 0, 0)
        self.context.set_line_width(0.01)
        self.context.stroke()
        self.sign_id(mod_id)
        self.movement_stack.append(
            [
                self.x_pos,
                self.y_pos,
                self.orientation,
                self.rotating_orientation,
            ]
        )

    def draw_hinge(self, mod_id: int | str):
        """Draw a hinge (blue) on the previous object"""

        self.context.rectangle(self.x_pos, self.y_pos, 1, 1)
        if self.rotating_orientation == 0:
            self.context.set_source_rgb(1.0, 0.4, 0.4)
        else:
            self.context.set_source_rgb(1, 0, 0)
        self.context.fill_preserve()
        self.context.set_source_rgb(0, 0, 0)
        self.context.set_line_width(0.01)
        self.context.stroke()
        self.calculate_orientation()
        self.sign_id(mod_id)
        self.movement_stack.append(
            [
                self.x_pos,
                self.y_pos,
                self.orientation,
                self.rotating_orientation,
            ]
        )

    def draw_module(self, mod_id: int | str):
        """Draw a module (red) on the previous object"""
        self.context.rectangle(self.x_pos, self.y_pos, 1, 1)
        self.context.set_source_rgb(0, 0, 1)
        self.context.fill_preserve()
        self.context.set_source_rgb(0, 0, 0)
        self.context.set_line_width(0.01)
        self.context.stroke()
        self.calculate_orientation()
        self.sign_id(mod_id)
        self.movement_stack.append(
            [
                self.x_pos,
                self.y_pos,
                self.orientation,
                self.rotating_orientation,
            ]
        )

    def calculate_sensor_rectangle_position(self):
        """Calculate squeezed sensor rectangle position based on current orientation and last movement direction"""
        if self.previous_move is None:
            return self.x_pos, self.y_pos + 0.9, 1, 0.1
        else:
            match self.previous_move.to_angle() + self.orientation.to_angle():
                case RightAngles.RAD_0:
                    return self.x_pos, self.y_pos + 0.9, 1, 0.1
                case RightAngles.RAD_HALFPI:
                    return self.x_pos, self.y_pos, 0.1, 1
                case RightAngles.RAD_PI:
                    return self.x_pos, self.y_pos, 1, 0.1
                case RightAngles.RAD_ONEANDAHALFPI:
                    return self.x_pos + 0.9, self.y_pos, 0.1, 1

        raise ValueError("Cannot calculate sensor position: Invalid")

    def save_sensor_position(self):
        """Save sensor position in list"""
        x, y, x_scale, y_scale = self.calculate_sensor_rectangle_position()
        self.sensors.append([x, y, x_scale, y_scale])
        self.calculate_orientation()
        self.movement_stack.append(
            [
                self.x_pos,
                self.y_pos,
                self.orientation,
                self.rotating_orientation,
            ]
        )

    def draw_sensors(self):
        """Draw all sensors"""
        for sensor in self.sensors:
            self.context.rectangle(sensor[0], sensor[1], sensor[2], sensor[3])
            self.context.set_source_rgb(0.6, 0.6, 0.6)
            self.context.fill_preserve()
            self.context.set_source_rgb(0, 0, 0)
            self.context.set_line_width(0.01)
            self.context.stroke()

    def calculate_connector_to_parent_position(self):
        """Calculate position of connector node on canvas"""
        parent = self.movement_stack[-2]
        parent_orientation = parent[2]

        if self.previous_move is not None:
            match self.previous_move.to_angle() + parent_orientation.to_angle():
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

    def save_png(self, file_name: Path):
        """Store image representation of canvas"""
        self.surface.write_to_png(str(file_name))

    def reset_canvas(self):
        """Reset canvas variables to default values"""
        self.x_pos = 0
        self.y_pos = 0
        self.orientation = Directions.FRONT
        self.previous_move = None
        self.movement_stack = []
        self.sensors = []
        self.rotating_orientation = 0
