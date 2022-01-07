from abc import ABC, abstractmethod


class Transaction:
    @abstractmethod
    def __enter__(self):
        pass

    @abstractmethod
    def __exit__(self, exc_type, exc_value, exc_traceback):
        pass
