from nca.core.abstract.structural.tree import Tree2D
from nca.core.abstract.structural.tree.tree_helper import Orientation


def depth_first_search(visited, graph, node):
    if node not in visited:
        visited.append(node)
        for neighbour, element in graph[node]:
            depth_first_search(visited, graph, element)


root = Tree2D()
root.append(Orientation.TOP)

child = Tree2D()
child.append(Orientation.LEFT)
child.append(Orientation.RIGHT)
child.append(Orientation.DOWN)
root.append(Orientation.DOWN)
depth_first_search(set(), root.graph(), root)
