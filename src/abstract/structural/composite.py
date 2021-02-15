from abc import ABC, abstractmethod
from typing import List


class Component(ABC):

    def __init__(self):
        pass

    @abstractmethod
    def execute(self):
        pass


class Leaf(Component, ABC):

    def __init__(self):
        super().__init__()

    def execute(self):
        pass


class Composite(Component, ABC):

    def __init__(self, children: List[Component] = None):
        super().__init__()
        self.children: List[Component] = children if children is not None else []

    def add(self, component: Component):
        self.children.append(component)

    def remove(self, component: Component):
        if component in self.children:
            self.children.remove(component)

    def get_children(self):
        return self.children

    def execute(self):
        pass
