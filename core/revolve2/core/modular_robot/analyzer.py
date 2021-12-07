from typing import List, Optional, Tuple

from .analyzer_module import AnalyzerModule
from .body import Body
from .module import Module


class Analyzer:
    _core: AnalyzerModule
    _active_hinges: List[AnalyzerModule]

    def __init__(self, body: Body):
        self._active_hinges = []
        (self._core, _) = self._init_tree_node(body.core, 0, None, None)

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

        for child_index, child in enumerate(module.children):
            if child is not None:
                (analyzer_child, next_id) = self._init_tree_node(
                    child, next_id, analyzer_module, child_index
                )
                analyzer_module.children[child_index] = analyzer_child

        return (analyzer_module, next_id)

    @property
    def active_hinges(self) -> List[AnalyzerModule]:
        """
        Get the active hinges in this modular robot.
        """
        return self._active_hinges

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
