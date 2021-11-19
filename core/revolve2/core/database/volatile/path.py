from dataclasses import dataclass
from typing import List

from ..path import Path as PathBase
from .db_item import DbItem


@dataclass
class Path(PathBase):
    item: DbItem
