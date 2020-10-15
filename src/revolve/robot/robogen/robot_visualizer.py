import math
import matplotlib as mpl
from matplotlib import pyplot, cm, colors

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

from nca.core.abstract.structural.tree.tree_helper import Coordinate3D
from revolve.robot.body.robogen_body import RobogenBody
from revolve.robot.robogen.robogen_grammar import RobogenSymbol, RobogenModule


def generate_matrix(body_modules):
    max_x, max_y, min_x, min_y = -math.inf, -math.inf, math.inf, math.inf
    for body_element in body_modules:
        if body_element.coordinate.x > max_x:
            max_x = body_element.coordinate.x
        if body_element.coordinate.x < min_x:
            min_x = body_element.coordinate.x
        if body_element.coordinate.y > max_y:
            max_y = body_element.coordinate.y
        if body_element.coordinate.y < min_y:
            min_y = body_element.coordinate.y

    length = max_x + abs(min_x) + 1
    height = max_y + abs(min_y) + 1

    # ensure that the length and height are equally sized for the 3D plot to be visible.
    if length > height:
        height = length
    elif height > length:
        length = height

    matrix_coordinate = Coordinate3D(int(- min_x), int(- min_y), 0)
    body_matrix = np.zeros([length, height])
    for body_element in body_modules:
        coordinate = body_element.coordinate + matrix_coordinate
        body_matrix[coordinate.x, coordinate.y] = body_element.symbol.value
    return body_matrix


def show(body_matrix):
    # make a color map of fixed colors
    number_of_modules = len(RobogenSymbol.modules()) + 2
    cmap = colors.ListedColormap(['white', 'yellow', 'blue', 'red', 'pink'])
    bounds = range(number_of_modules + 1)
    norm = mpl.colors.BoundaryNorm(bounds, cmap.N)

    # tell imshow about color map so that only set colors are used
    img = pyplot.imshow(body_matrix, interpolation='nearest', cmap=cmap, norm=norm)

    # make a color bar
    cbar = pyplot.colorbar(img, cmap=cmap, norm=norm, boundaries=bounds, )
    module_array = [str(module.name) for module in RobogenSymbol.modules()]
    module_array.insert(0, "Core")
    module_array.insert(0, "Empty")
    module_array.append("")

    cbar.ax.get_yaxis().set_ticks([])
    for j, lab in enumerate(module_array):
        cbar.ax.text(3, (2 * j + 1) / 2.0, lab, ha='center', va='center')
    cbar.ax.get_yaxis().labelpad = 50

    pyplot.show()