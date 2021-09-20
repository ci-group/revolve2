import math
import matplotlib as mpl
from matplotlib import pyplot, colors
import matplotlib.pyplot as plt

import numpy as np

from revolve2.abstract.structural.tree.tree_helper import Coordinate3D
from revolve2.revolve.robot.body.robogen.robogen_grammar import RobogenSymbol


def generate_matrix(body_modules):
    connections = []
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
        if body_element.symbol is not RobogenSymbol.MODULE_CORE:
            connection_coordinate = coordinate - (body_element.orientation.value / 2.0)
            connections.append([connection_coordinate.x, connection_coordinate.y])
    return body_matrix, np.array(connections), height, length


def show(body_matrix, connections, length, height, path=None):
    # make a color map of fixed colors
    number_of_modules = len(RobogenSymbol.modules()) + 2
    cmap = colors.ListedColormap(['white', 'yellow', 'blue', 'red', 'pink'])
    bounds = range(number_of_modules + 1)
    norm = mpl.colors.BoundaryNorm(bounds, cmap.N)

    # tell imshow about color map so that only set colors are used
    img = plt.imshow(body_matrix, interpolation='nearest', cmap=cmap, norm=norm, origin='lower')

    # make a color bar
    module_array = [str(module.name) for module in RobogenSymbol.modules()]
    module_array.insert(0, "Core")
    module_array.insert(0, "Empty")
    module_array.append("")
    cbar = plt.colorbar(img)

    cbar.ax.get_yaxis().set_ticks([])
    for j, lab in enumerate(module_array):
        cbar.ax.text(3, (2 * j + 1) / 2.0, lab, ha='center', va='center')
    cbar.ax.get_yaxis().labelpad = 50

    ax = plt.gca()

    # Major ticks
    ax.set_xticks(np.arange(0, length, 1))
    ax.set_yticks(np.arange(0, height, 1))

    # Minor ticks
    ax.set_xticks(np.arange(-.5, length, 1), minor=True)
    ax.set_yticks(np.arange(-.5, height, 1), minor=True)

    # Gridlines based on minor ticks
    ax.grid(which='minor', color="white", linewidth=1)
    if len(connections) > 0:
        plt.scatter(connections[:, 1], connections[:, 0], color="black")

    if path is None:
        plt.show()
    else:
        plt.savefig(path)
        plt.close()
