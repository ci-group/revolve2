from .._serializable_list import SerializableList


class Parameters(
    SerializableList[float], table_name="parameters", value_column_name="parameter"
):
    """A serializable list of parameters."""

    pass
