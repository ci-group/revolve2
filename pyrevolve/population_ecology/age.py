
class Age:

    def __init__(self):
        self.generations: int = 0
        self.no_improvement: int = 0

    def update(self, improved: bool):
        self.generations += 1

        # increment everytime, but only reset when improvements occur.
        self.no_improvement += 1
        if not improved:
            self.no_improvement = 0

    def serialize(self):
        pass
