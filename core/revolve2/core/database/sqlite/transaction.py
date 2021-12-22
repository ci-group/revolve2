import logging
from typing import Any

from ..transaction import Transaction as TransactionBase


class Transaction(TransactionBase):
    _session: Any  # TODO sqlalchemy typing bad

    def __init__(self, session: Any) -> None:
        self._session = session
