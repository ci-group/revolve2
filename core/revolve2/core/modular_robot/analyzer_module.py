from __future__ import annotations

from typing import List, Optional

from .module import Module


class AnalyzerModule:
    _id: int
    _module: Module
    _children: List[Optional[AnalyzerModule]]
    _parent: Optional[AnalyzerModule]

    def __init__(self, module: Module, id: int, parent: Optional[AnalyzerModule]):
        self._module = module
        self._id = id
        self._children = [None] * module.num_children
        self._parent = parent

    def get_child(self, index: int) -> Optional[AnalyzerModule]:
        return self._children[index]

    def set_child(self, index: int, module: AnalyzerModule) -> None:
        self._children[index] = module

    @property
    def num_children(self) -> int:
        return len(self._children)

    @property
    def parent(self) -> Optional[AnalyzerModule]:
        return self._parent

    @property
    def type(self) -> Module.Type:
        return self.module.type

    @property
    def id(self) -> int:
        return self._id

    @property
    def module(self) -> Module:
        return self._module
