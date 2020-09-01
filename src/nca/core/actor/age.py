
class Age:

    def __init__(self):
        self.generations: int = 0
        # self.time = TimeInterval()

    def update(self):
        self.generations += 1


class GenerationalAge(Age):

    def __init__(self):
        super().__init__()
        self.no_improvement_count = 0

    def increment(self, improved: bool):
        super().update()

        if not improved:
            self.no_improvement_count = 0
        else:
            self.no_improvement_count += 1
