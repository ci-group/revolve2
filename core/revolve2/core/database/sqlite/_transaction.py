from types import TracebackType
from typing import Any, Optional, Type

from sqlalchemy.orm import Session

from .._transaction import Transaction as TransactionBase


class Transaction(TransactionBase):
    _session: Session

    def __init__(self, session: Session) -> None:
        self._session = session

    def __exit__(
        self,
        exc_type: Optional[Type[BaseException]],
        exc_value: Optional[BaseException],
        exc_traceback: Optional[TracebackType],
    ) -> None:
        if exc_type is None:
            self._session.commit()
        else:
            self._session.rollback()
        self._session.close()
