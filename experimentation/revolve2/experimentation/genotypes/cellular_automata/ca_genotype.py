import numpy as np
import random


class CAGenotype:
    init_state = []
    core_position = [0, 0]
    rule_set = {}
    iterations = 1

    ca_grid = []

    def set_params(self, init_state, iterations, rule_set):
        self.init_state = init_state
        self.iterations = iterations
        self.rule_set = rule_set

    def generate_body(self):
        domain = self.init_state
        for i in range(self.iterations):
            domain = self.update_grid(domain)
            print(domain)

        self.ca_grid = domain.copy()

    def update_grid(self, grid):
        rows, cols = len(grid), len(grid[0])
        domain_next = grid.copy()
        for i in range(1, rows - 1):
            for j in range(1, cols - 1):
                neighborhood = (
                    grid[i + 1][j],
                    grid[i - 1][j],
                    grid[i][j + 1],
                    grid[i][j - 1],
                )

                if self.rule_set.get(neighborhood) is not None:
                    cell_value = self.rule_set.get(neighborhood)
                    domain_next[i][j] = cell_value

        return domain_next

    def get_full_grid(self):
        return self.ca_grid

    def get_separate_grids(self):
        rows, cols = len(self.ca_grid), len(self.ca_grid[0])
        module_grid = np.zeros((rows, cols))
        orientation_grid = np.zeros((rows, cols))
        for i in range(rows):
            for j in range(cols):
                module_grid[i][j] = self.ca_grid[i][j]
                orientation_grid[i][j] = self.ca_grid[i][j]

        return module_grid, orientation_grid

    # Temporary function for testing
    def set_core(self, i, j):
        self.core_position = [i, j]
        self.ca_grid[i][j] = 3

    def mutate(self):
        possible_values = [0.0, 1.0, 2.0]
        new_key = tuple(random.choice(possible_values) for _ in range(4))
        new_value = random.choice(possible_values[1:])

        # Choose an existing key to replace
        existing_key = random.choice(list(self.rule_set.keys()))
        self.rule_set[new_key] = new_value
        del self.rule_set[existing_key]

    def generate_random_genotype(self, n):
        possible_values = [0.0, 1.0, 2.0]
        gene = {}

        for i in range(n):
            # Generate n random key value pairs
            new_key = tuple(random.choice(possible_values) for _ in range(4))
            new_value = random.choice(possible_values[1:])
            gene[new_key] = new_value

        self.rule_set = gene
