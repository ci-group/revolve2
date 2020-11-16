
class Statistics:

    def __init__(self):
        self.maximum_value = []
        self.upper_quartile = []
        self.median_value = []
        self.lower_quartile = []
        self.minimum_value = []

    def log(self, max, third, median, first, min):
        self.maximum_value.append(max)
        self.upper_quartile.append(third)
        self.median_value.append(median)
        self.lower_quartile.append(first)
        self.minimum_value.append(min)