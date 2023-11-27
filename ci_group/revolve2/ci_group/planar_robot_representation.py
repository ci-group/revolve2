from revolve2.modular_robot import ModularRobot
from revolve2.modular_robot.body import Module
from revolve2.modular_robot.body.base import Body, Core, ActiveHinge, Brick
import cairo
import os
from pyrr import Quaternion, Vector3
import numpy as np
import time
from numpy.typing import NDArray


def __mk_path() -> str:
    path = f"planar_robot_representations_{time.time()}"
    print(f"Saving images to: {path}")
    if not os.path.exists(path):
        os.mkdir(path)
    return path


def draw_robots(robots: list[ModularRobot] | list[Body], scale: int = 100, path: str | None = None) -> None:
    if not path:
        path = __mk_path()

    for robot in robots:
        draw_robot(robot, scale, path)


def draw_robot(robot: ModularRobot | Body, scale: int = 100, path: str | None = None) -> None:
    """
    Draw a 2D representation for a modular robots body.

    :param robot: Supply the robot as a ModularRobot object, or the body directly as a Body object.
    :param scale: Allows to set the size of the drawing.
    :param path: The path to save images to.
    """
    if not path:
        path = __mk_path()

    body = robot if isinstance(robot, Body) else robot.body
    body_grid, core_position = body.to_grid()
    x, y, _ = body_grid.shape

    image = cairo.ImageSurface(cairo.FORMAT_ARGB32, x * scale, y * scale)
    context = cairo.Context(image)
    context.scale(scale, scale)

    cx, cy, _ = tuple(core_position)
    _draw_module(
        module=body.core,
        position=(cx, cy),
        orientation=Quaternion(),
        context=context,
    )
    _save_png(image, path)


def _draw_module(module: Module,
                 position: tuple[int, int],
                 orientation: Quaternion,
                 context: cairo.Context,
                 print_id: bool = False) -> None:
    """
    Draw a module onto the canvas.


    :param module: The module.
    :param position: The position on the canvas.
    :param orientation: The orientation to draw in.
    :param context: The context to draw it on.
    :param print_id: If the modules id should be drawn as well.
    :return: The context.
    :raises Exception: If the module cant be drawn.
    """
    print(f"drawing {type(module)} onto {position}")
    x, y = position
    context.rectangle(x, y, 1, 1)  # draw module object

    match module:
        case Core():
            context.set_source_rgb(255, 255, 0)  # Yellow
        case ActiveHinge():
            rotation = module.rotation
            context.set_source_rgb(1, 0, 0)  # Red
            if rotation == 0:
                context.set_source_rgb(1.0, 0.4, 0.4)  # Flesh Color
        case Brick():
            context.set_source_rgb(0, 0, 1)  # Blue
        case _:
            raise Exception(f"Module of type {type(module)} has no defined structure for drawing.")

    # default operation for every module
    context.fill_preserve()
    context.set_source_rgb(0, 0, 0)
    context.set_line_width(0.01)
    context.stroke()

    if print_id:
        # print module id onto canvas
        context.set_font_size(0.3)
        context.move_to(x, y + 0.4)
        context.set_source_rgb(0, 0, 0)
        context.show_text(str(module.uuid))
        context.stroke()

    for key, child in module.children.items():
        new_orient = module.attachment_points.get(key).orientation * orientation

        rot = __make_rot_matrix(new_orient.angle)
        offset = np.round(np.dot(rot, np.array([1, 0])))
        new_pos = position[0] + offset[0], position[1] + offset[1]

        _draw_module(
            module=child,
            position=new_pos,
            context=context,
            orientation=new_orient,
        )


def _save_png(image: cairo.ImageSurface, path: str) -> None:
    """
    Save the image representation of a robot as png.

    :param image: The image.
    :param path: The path to save the image to.
    """
    image.write_to_png(f"{path}/robot_2d_{str(hash(image))}.png")


def __make_rot_matrix(angle: float) -> NDArray[np.float_]:
    rot_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                           [np.sin(angle), np.cos(angle)]])
    return rot_matrix
