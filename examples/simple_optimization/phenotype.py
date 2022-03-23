from typing import List


class Phenotype(List[bool]):
    items: List[bool]

    def __init__(self, items: List[bool]) -> None:
        self.items = items
