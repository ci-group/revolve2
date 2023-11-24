from dataclasses import dataclass
from typing import Iterator, Optional, Self, Set, Tuple, List, TypeVar, cast

import numpy as np
from numpy.random import Generator
from revolve2.modular_robot import Body, Core, Directions

from revolve2.experimentation.genotypes.protocols import GenotypeInitParams, IGenotype
from revolve2.experimentation.genotypes.tree import (
    Nodes_t,
    Node,
    CoreNode,
    BrickNode,
    ActiveHingeNode,
)

Nodes_gT = TypeVar("Nodes_gT", bound=Nodes_t)
Nodes_g = TypeVar("Nodes_g", bound=Node)


@dataclass
class TreeInitParameters(GenotypeInitParams):
    max_depth: int


class TreeGenotype(IGenotype):
    VALID_CHILDREN = [BrickNode, ActiveHingeNode]

    def __init__(
        self,
        params: TreeInitParameters,
        tree: CoreNode = CoreNode((d, None) for d in CoreNode.valid_attatchments()),
    ) -> None:
        self._params = params
        self._tree = tree
        self._prune_overlap()

    @classmethod
    def random_subtree(
        cls, node_t: type[Nodes_gT], depth: int, rng: Generator
    ) -> Nodes_gT:
        # Monads hide in little corners. Watch out!
        return node_t(
            (d, cls.random_subtree(childtype, depth - 1, rng))
            for d in node_t.valid_attatchments()
            if depth >= 0
            if (childtype := rng.choice(cls.VALID_CHILDREN + [None])) is not None
        )

    @classmethod
    def random(cls, params: TreeInitParameters, rng: Generator) -> Self:
        return cls(params, cls.random_subtree(CoreNode, params.max_depth, rng))

    def develop(self) -> Body:
        body = Body()
        body.core = cast(Core, self._tree.to_module())
        body.finalize()
        return body

    def copy(self) -> Self:
        return self.__class__(self._params, self._tree.copy())

    def crossover(self, rng: Generator, __o: Self) -> Self:
        selftree, otree = self._tree.copy(), __o._tree.copy()
        tree = CoreNode(
            (d, selftree.get_child(d))
            if d in rng.choice(Directions.values(), 2)
            else (d, otree.get_child(d))
            for d in CoreNode.valid_attatchments()
        )
        return self.__class__(self._params, tree)

    def mutate(self, rng: Generator) -> Self:
        ret = self.copy()
        node, depth = rng.choice(list(ret.nodes()))
        node.set_child(
            self.random_subtree(
                rng.choice(self.VALID_CHILDREN), self._params.max_depth - depth, rng
            ),
            rng.choice(node.valid_attatchments()),
        )
        return ret

    def nodes(self) -> Iterator[Tuple[Node, int]]:
        def inner(node: Node, depth: int) -> Iterator[Tuple[Node, int]]:
            yield node, depth
            for _, child in node.children:
                yield from inner(child, depth + 1)

        return inner(self._tree, 0)

    def _prune_overlap(self) -> None:
        Location = Tuple[int, int]
        self._occupied_slots: Set[Location] = set()

        # Yes, this can be done with vector algebra,
        # However, counterargument: I am lazy
        def add_angle(location: Location, d: Directions) -> Location:
            x, y = location
            match d:
                case Directions.FRONT:
                    return x + 1, y
                case Directions.BACK:
                    return x - 1, y
                case Directions.RIGHT:
                    return x, y + 1
                case Directions.LEFT:
                    return x, y - 1

        def inner(node: Nodes_g, child_d: Directions, location: Location) -> Nodes_g:
            new_children: List[Tuple[Directions, Node]] = []
            for child_d, child in node.children:
                new_dir = Directions.from_angle(child_d.to_angle() + child_d.to_angle())
                loc = add_angle(location, new_dir)
                if not loc in self._occupied_slots:
                    new_children.append((child_d, inner(child, new_dir, loc)))
                    self._occupied_slots.add(loc)

            return node.__class__(new_children)

        self._tree = inner(self._tree, Directions.FRONT, (0, 0))
        del self._occupied_slots

    def print_tree(self) -> None:
        def inner(node: Node, type_str: str, depth: int) -> None:
            print(f"{'│   ' * depth}{type_str} - {node.__class__.__name__}")
            for direction, child in node.children:
                inner(child, direction.name, depth + 1)

        inner(self._tree, "HEAD ", 0)
