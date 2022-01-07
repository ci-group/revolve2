from typing import Any

from sqlalchemy.orm import Session

from ..transaction import Transaction as TransactionBase


class Transaction(TransactionBase):
    _session: Session

    def __init__(self, session: Session) -> None:
        self._session = session

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        if exc_type is None:
            self._session.commit()
        else:
            self._session.rollback()
        self._session.close()
