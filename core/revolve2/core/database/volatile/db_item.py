from dataclasses import dataclass
from typing import Dict, List, Union


@dataclass
class DbItem:
    item: "DbType"
    deleted: bool


DbType = Union[int, float, str, bytes, None, List["DbItem"], Dict[str, "DbItem"]]
