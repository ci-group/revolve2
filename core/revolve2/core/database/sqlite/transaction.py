from typing import Any

from sqlalchemy.orm import Session

from ..transaction import Transaction as TransactionBase


class Transaction(TransactionBase):
    _session: Session

    def __init__(self, session: Session) -> None:
        self._session = session
