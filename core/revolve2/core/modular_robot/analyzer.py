from typing import List, Tuple

from .analyzer_module import AnalyzerModule
from .body import Body
from .module import Module


class Analyzer:
    _core: AnalyzerModule
    _active_hinges: List[AnalyzerModule]

    def __init__(self, body: Body):
        self._active_hinges = []
        (self._core, _) = self._init_tree_node(body.core, 0)

    def _init_tree_node(
        self, module: Module, next_id: int
    ) -> Tuple[AnalyzerModule, int]:
        analyzer_module = AnalyzerModule(module, next_id)
        next_id += 1
        for child_index in range(module.num_children):
            child = module.get_child(child_index)
            if child is not None:
                (analyzer_child, next_id) = self._init_tree_node(child.module, next_id)
                analyzer_module.set_child(child_index, analyzer_child)

        if module.type == Module.Type.ACTIVE_HINGE:
            self._active_hinges.append(analyzer_module)

        return (analyzer_module, next_id)

    @property
    def active_hinges(self) -> List[AnalyzerModule]:
        return self._active_hinges

    def neighbours(
        self, module: AnalyzerModule, within_range: int
    ) -> List[AnalyzerModule]:
        raise NotImplementedError()

    @property
    def core(self) -> AnalyzerModule:
        return self._core
