from dataclasses import dataclass
from typing import List


@dataclass
class Phenotype(List[bool]):
    items: List[bool]
