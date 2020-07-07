from typing import List


class Iterator():

    def __init__(self, collection: List[object]):
        self.collection: List[object] = collection

    def __iter__(self):
        return self

    def add(self, element: object):
        self.collection.append(element)

    def remove(self, element: object):
        self.collection.remove(element)

    def __next__(self):
        next(self.collection)
