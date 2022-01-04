import math
from typing import List, Optional, Tuple

from pyrr import Quaternion, Vector3

from .active_hinge import ActiveHinge
from .analyzer_module import AnalyzerModule
from .body import Body
from .brick import Brick
from .core import Core
from .module import Module


class Analyzer:
    _core: AnalyzerModule
    _num_modules: int
    _active_hinges: List[AnalyzerModule]
    _bricks: List[AnalyzerModule]

    def __init__(self, body: Body):
        self._active_hinges = []
        self._bricks = []
        (self._core, self._num_modules) = self._init_tree_node(body.core, 0, None, None)

    def _init_tree_node(
        self,
        module: Module,
        next_id: int,
        parent: Optional[AnalyzerModule],
        parent_child_index: Optional[int],
    ) -> Tuple[AnalyzerModule, int]:
        analyzer_module = AnalyzerModule(module, next_id, parent, parent_child_index)
        next_id += 1

        if module.type == Module.Type.ACTIVE_HINGE:
            self._active_hinges.append(analyzer_module)
        elif module.type == Module.Type.BRICK:
            self._bricks.append(analyzer_module)

        for child_index, child in enumerate(module.children):
            if child is not None:
                (analyzer_child, next_id) = self._init_tree_node(
                    child, next_id, analyzer_module, child_index
                )
                analyzer_module.children[child_index] = analyzer_child

        return (analyzer_module, next_id)

    @property
    def num_modules(self) -> int:
        return self._num_modules

    @property
    def active_hinges(self) -> List[AnalyzerModule]:
        """
        Get the active hinges in this modular robot.
        """
        return self._active_hinges

    @property
    def bricks(self) -> List[AnalyzerModule]:
        """
        Get the bricks in this modular robot.
        """
        return self._bricks

    def neighbours(
        self, module: AnalyzerModule, within_range: int
    ) -> List[AnalyzerModule]:
        out_neighbours: List[AnalyzerModule] = []

        open_nodes: List[Tuple[AnalyzerModule, Optional[AnalyzerModule]]] = [
            (module, None)
        ]  # (module, came_from)

        for _ in range(within_range):
            new_open_nodes: List[Tuple[AnalyzerModule, Optional[AnalyzerModule]]] = []
            for (open_node, came_from) in open_nodes:
                neighbours = [
                    mod
                    for mod in open_node.children + [open_node.parent]
                    if mod is not None
                    and (came_from is None or mod.id is not came_from.id)
                ]
                out_neighbours += neighbours
                new_open_nodes += list(zip(neighbours, [open_node] * len(neighbours)))
            open_nodes = new_open_nodes

        return out_neighbours

    @property
    def core(self) -> AnalyzerModule:
        return self._core

    def size(self) -> Tuple[float, float, float, float, float, float]:
        """
        Get size of robot measured in number of modules.
        -x, +x, -y, +y, -z, +z
        with respect to the core.
        All angles must be multiples of 90 degrees.
        """
        size = self._recursive_size(self._core, Vector3([0, 0, 0]), Quaternion())
        return (
            round(size[0]),
            round(size[1]),
            round(size[2]),
            round(size[3]),
            round(size[4]),
            round(size[5]),
        )

    def _recursive_size(
        self, module: AnalyzerModule, position: Vector3, orientation: Quaternion
    ) -> Tuple[int, int, int, int, int, int]:
        if module.type == Module.Type.CORE:
            # for (child_index, angle) in [
            children = [
                (Core.FRONT, 0.0),
                (Core.BACK, math.pi),
                (Core.LEFT, math.pi / 2.0),
                (Core.RIGHT, math.pi / 2.0 * 3),
            ]
        elif module.type == Module.Type.BRICK:
            children = [
                (Brick.FRONT, 0.0),
                (Brick.LEFT, math.pi / 2.0),
                (Brick.RIGHT, math.pi / 2.0 * 3),
            ]
        elif module.type == Module.Type.ACTIVE_HINGE:
            children = [(ActiveHinge.ATTACHMENT_INDEX, 0.0)]

        sizes = [
            (position.x, position.x, position.y, position.y, position.z, position.z)
        ] + [
            self._recursive_size(
                child, position + quat * Vector3([1.0, 0.0, 0.0]), quat
            )
            for child, quat in [
                (
                    module.children[index],
                    orientation
                    * Quaternion.from_eulers([0.0, 0.0, angle])
                    * Quaternion.from_eulers(
                        [module.children[index].module.rotation, 0.0, 0.0]
                    ),
                )
                for (index, angle) in children
                if module.children[index] is not None
            ]
        ]

        minx = min(sizes, key=lambda size: size[0])[0]
        maxx = max(sizes, key=lambda size: size[1])[1]
        miny = min(sizes, key=lambda size: size[2])[2]
        maxy = max(sizes, key=lambda size: size[3])[3]
        minz = min(sizes, key=lambda size: size[4])[4]
        maxz = max(sizes, key=lambda size: size[5])[5]

        return (minx, maxx, miny, maxy, minz, maxz)
