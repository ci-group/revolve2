from nca.core.abstract.structural.tree import Tree2D
from nca.core.abstract.structural.tree.tree_helper import Orientation


def depth_first_search(visited, graph, node):
    if node not in visited:
        visited.add(node)
        for neighbour, element in graph[node]:
            depth_first_search(visited, graph, element)


root = Tree2D()
root.add(Orientation.TOP)

child = Tree2D()
child.add(Orientation.LEFT)
child.add(Orientation.RIGHT)
child.add(Orientation.DOWN)
root.add(Orientation.DOWN)
print(root.graph().keys())
depth_first_search(set(), root.graph(), root)
