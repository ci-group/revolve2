from typing import List, Mapping, Union

Serialized = Union[List["Serialized"], Mapping[str, "Serialized"], str, float, int]
