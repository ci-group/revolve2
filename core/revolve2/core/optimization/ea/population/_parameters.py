from revolve2.core.database import SerializableFrozenList


class Parameters(
    SerializableFrozenList[float],
    table_name="parameters",
    value_column_name="parameter",
):
    """A serializable list of parameters."""

    pass
