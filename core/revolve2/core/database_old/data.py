from typing import Dict, List, Union

from .dynamic_list import DynamicList

Data = Union[List["Data"], Dict[str, "Data"], str, float, int, bytes, None, DynamicList]
