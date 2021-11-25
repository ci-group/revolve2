from dataclasses import dataclass
from typing import List

from ..path import Path as PathBase


@dataclass
class Path(PathBase):
    path: str
