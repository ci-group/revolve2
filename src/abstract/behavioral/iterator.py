from typing import List


class Iterator:

    def __init__(self, collection: List[object] = None):
        if collection is None:
            collection = []

        self.collection: List[object] = collection
        self.index = 0

    def add(self, element: object):
        self.collection.append(element)

    def remove(self, element: object):
        self.collection.remove(element)

    def __next__(self) -> object:
        try:
            agent = self.collection[self.index]
            self.index += 1
        except IndexError:
            raise StopIteration()

        return agent

    def __len__(self):
        return len(self.collection)

    def __getitem__(self, index):
        return self.collection[index]

    def __iter__(self):
        self.index = 0
        return self
