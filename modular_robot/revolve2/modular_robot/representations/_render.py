from pathlib import Path
from ._canvas import Canvas
from ._grid import Grid
from revolve2.modular_robot import Core, ActiveHinge, Brick, Directions, Module


class Render:
    def __init__(self):
        """Instantiate grid"""
        self.grid = Grid()

    def parse_body_to_draw(self, canvas: Canvas, module: Module, slot: Directions):
        """
        Parse the body to the canvas to draw the png
        @param canvas: instance of the Canvas class
        @param module: body of the robot
        @param slot: parent slot of module
        """
        # TODO: map slots to enumerators

        if isinstance(module, Core):
            canvas.draw_controller(module.id)
        elif isinstance(module, ActiveHinge):
            canvas.move_by_slot(slot)
            canvas.rotating_orientation = 0
            canvas.draw_hinge(module.id)
            canvas.draw_connector_to_parent()
        elif isinstance(module, Brick):
            canvas.move_by_slot(slot)
            canvas.rotating_orientation = 0
            canvas.draw_module(module.id)
            canvas.draw_connector_to_parent()
        else:
            raise ValueError(f"{module} is of an unknown module type")

        if module.has_children():
            # Traverse children of element to draw on canvas
            for core_slot, child_module in enumerate(module.children):
                if child_module is None:
                    continue
                self.parse_body_to_draw(canvas, child_module, Directions(core_slot))
            canvas.move_back()
        else:
            # Element has no children, move back to previous state
            canvas.move_back()

    def traverse_path_of_robot(
        self, module: Module, slot: Directions, include_sensors: bool = True
    ):
        """
        Traverse path of robot to obtain visited coordinates
        @param module: body of the robot
        @param slot: attachment of parent slot
        @param include_sensors: add sensors to visisted_cooridnates if True
        """
        if isinstance(module, ActiveHinge) or isinstance(module, Brick):
            self.grid.move_by_slot(slot)
            self.grid.add_to_visited(include_sensors, False)
        if module.has_children():
            # Traverse path of children of module
            for core_slot, child_module in enumerate(module.children):
                if child_module is None:
                    continue
                self.traverse_path_of_robot(
                    child_module, Directions(core_slot), include_sensors
                )
            self.grid.move_back()
        else:
            # Element has no children, move back to previous state
            self.grid.move_back()

    def render_robot(self, body_core: Module, image_path: Path):
        """
        Render robot and save image file
        @param body: body of robot
        @param image_path: file path for saving image
        """
        # Calculate dimensions of drawing and core position
        self.traverse_path_of_robot(body_core, Directions.FRONT)
        self.grid.calculate_grid_dimensions()
        core_position = self.grid.calculate_core_position()

        # Draw canvas
        if isinstance(self.grid.width, int) and isinstance(self.grid.height, int):
            cv = Canvas(self.grid.width, self.grid.height, 100)
            cv.set_position(core_position[0], core_position[1])
        else:
            raise RuntimeError("Error while configuring gird")

        # Draw body of robot
        self.parse_body_to_draw(cv, body_core, Directions.FRONT)

        # Draw sensors after, so that they don't get overdrawn
        # cv.draw_sensors()

        cv.save_png(image_path)

        # Reset variables to default values
        cv.reset_canvas()
        self.grid.reset_grid()
