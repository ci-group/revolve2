"""
Basic test of classical cellular automata
Just to test the update function without networks
"""

import numpy as np
import matplotlib.pyplot as plt

# Function representing update rules for Cellular Automata
def custom_update(grid, possible_values):
    dsize = grid.shape[0]
    domain_next = grid.copy()

    for i in range(1, dsize - 1):
        for j in range(1, dsize - 1):
            inp = [grid[i][j], grid[i + 1][j], grid[i - 1][j],
                   grid[i][j + 1], grid[i][j - 1]]

            # Apply custom update rules
            # You can define your own rules based on inp and possible_values
            # For example, let's assume a rule that averages neighboring values
            average_value = sum(inp) / len(inp)
            domain_next[i][j] = min(possible_values, key=lambda x: abs(x - average_value))

    return domain_next

# Initialize the grid with random values from a specified list of possible values
dsize = 20
possible_values = [0, 1, 2, 3]  # Specify the possible values
initial_grid = np.random.choice(possible_values, size=(dsize, dsize))

# Number of iterations
num_iterations = 10

# Create a figure with subplots for visualization
fig, axs = plt.subplots(1, num_iterations + 1, figsize=(15, 5))

# Plot the initial state
axs[0].imshow(initial_grid, cmap='viridis', vmin=min(possible_values), vmax=max(possible_values))
axs[0].set_title('Initial State')

# Iterate and update the state for each iteration
current_state = initial_grid
for i in range(1, num_iterations + 1):
    current_state = custom_update(current_state, possible_values)
    axs[i].imshow(current_state, cmap='viridis', vmin=min(possible_values), vmax=max(possible_values))
    axs[i].set_title(f'Iteration {i}')

plt.show()
