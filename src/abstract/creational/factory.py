from abc import abstractmethod


class Factory:

    def __init__(self):
        pass

    @abstractmethod
    def create(self, *args, **kwargs) -> object:
        pass
