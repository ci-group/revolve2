from typing import Any, Dict, Union

from revolve2.core.database import SerializableStruct


class SerializableMeasures(SerializableStruct, table_name=None):
    """A class very similar to SerializableStruct with the added functionality of array indexing variables by name."""

    @classmethod
    def __init_subclass__(cls, /, table_name: str, **kwargs: Dict[str, Any]) -> None:
        """
        Initialize this object.

        :param table_name: Name of table in database.
        :param kwargs: Other arguments not specific to this class.
        """
        super().__init_subclass__(table_name, **kwargs)

    def __getitem__(self, key: str) -> Union[int, float, str, None]:
        """
        Get a measure.

        :param key: Name of the measure to get.
        :returns: The retrieved measure.
        """
        assert key in [
            col.name for col in self._columns
        ], f"measure {key} does not exist"
        val = getattr(self, key)
        assert val is None or type(val) == int or type(val) == float or type(val) == str
        return val

    def __setitem__(self, key: str, value: Union[int, float, str, None]) -> None:
        """
        Set a measure.

        :param key: Name of the measure to set.
        :param value: New value of the measure.
        """
        assert key in [
            col.name for col in self._columns
        ], "measure {key} does not exist"
        setattr(self, key, value)
