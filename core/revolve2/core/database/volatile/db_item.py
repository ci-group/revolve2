from dataclasses import dataclass
from typing import Dict, List, Tuple


@dataclass
class DbItem:
    item: "DbType"
    deleted: bool


DbType = Tuple[int, float, str, bytes, None, List["DbItem"], Dict[str, "DbItem"]]
