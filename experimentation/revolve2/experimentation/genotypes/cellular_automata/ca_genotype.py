import numpy as np



class CAGenotype:
    def __init__(self):
        init_state = []
        core_position = [0,0]
        iterations = 1
        ca_grid = []

    def set_params(self, init_state, iterations):
        self.init_state = init_state
        self.iterations = iterations

    def generate_body(self):
        domain = np.array(self.init_state).copy()
        for i in range(self.iterations):
            domain = self.update_grid(domain)
        
        self.ca_grid = domain.copy()

    @classmethod
    def update_grid(self, grid):
        rows, cols = grid.shape
        domain_next = grid.copy()
        for i in range(1, rows - 1):
            for j in range(1, cols - 1):
                neighbourhood = [grid[i][j], grid[i + 1][j], grid[i - 1][j],
                        grid[i][j + 1], grid[i][j - 1]]

                # a simple CA growth rule
                if np.sum(neighbourhood) == 1:
                    domain_next[i][j] = 1     

        return domain_next

    def get_grid(self):
        return self.ca_grid
    
    # temporary function for testing
    def set_core(self, i, j):
        self.core_position = [i, j]
        self.ca_grid[i][j] = 3

    