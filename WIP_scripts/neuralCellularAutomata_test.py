"""
Prototype test for neural cellular automata representation

Doesn't have life mask like in https://distill.pub/2020/growing-ca/ ... 
I will add it if the model turns out to be useful 

ISSUE: The core script seem to work properly, but NN doesn't actually learn to recreate an image
       Although this is perhaps not the main point of the code
"""


import numpy as np
import matplotlib.pyplot as plt
import torch
from torch import nn
import torch.optim as optim

class NeuralNetwork(nn.Module):
    def __init__(self):
        super().__init__()
        self.flatten = nn.Flatten()
        self.linear_relu_stack = nn.Sequential(
            nn.Linear(5, 32),
            nn.Tanh(),       # tanh is used according to the paper on CA from Karine
            nn.Linear(32, 3)  # number of output nodes = number of possible cell types
        )

    def forward(self, x):
        x = self.flatten(x)
        x = self.linear_relu_stack(x)
        return x

# function to visualize the activation map
def visualize_activation(grid, model):
    dsize = grid.shape[0]
    activation_map = np.zeros((dsize, dsize, 3))  # number of possible cell types

    for i in range(1, dsize - 1):
        for j in range(1, dsize - 1):
            inp = [grid[i][j], grid[i + 1][j], grid[i - 1][j],
                   grid[i][j + 1], grid[i][j - 1]]

            inp_tensor = torch.tensor([inp], dtype=torch.float32, requires_grad=True)
            out = model(inp_tensor)
            activation_map[i][j] = out.detach().numpy()

    return activation_map

# cellular automaton update function
def update_grid(grid, model):
    dsize = grid.shape[0]
    domain_next = grid.copy()

    for i in range(1, dsize - 1):
        for j in range(1, dsize - 1):
            inp = [grid[i][j], grid[i + 1][j], grid[i - 1][j],
                   grid[i][j + 1], grid[i][j - 1]]

            inp_tensor = torch.tensor([inp], dtype=torch.float32, requires_grad=True)
            out = model(inp_tensor)
            domain_next[i][j] = torch.argmax(out).item()

    return domain_next

if __name__ == '__main__':
    # domain to be learned
    dsize = 10
    target_domain = np.fromfunction(lambda x, y: ((x - dsize / 2)**2 + (y - dsize / 2)**2 <= (dsize / 4)**2).astype(int), (dsize, dsize))
    start_domain = np.random.randint(3, size=(dsize, dsize))  # Random initial grid with 3 classes

    model = NeuralNetwork()
    optimizer = optim.SGD(model.parameters(), lr=0.01)

    # training loop 
    num_train_iter = 50
    for j in range(num_train_iter):
        optimizer.zero_grad()

        # create domain based on rules from NN
        for i in range(10):
            domain_next = update_grid(start_domain, model)

        domain_tensor = torch.tensor(start_domain, dtype=torch.float32, requires_grad=True)
        domain_next_tensor = torch.tensor(domain_next, dtype=torch.float32, requires_grad=True)

        # measure of how close generated image is to the original
        similarity = torch.mean(torch.abs(domain_next_tensor - torch.tensor(target_domain, dtype=torch.float32)))
        loss = 1 - similarity

        loss.backward()
        optimizer.step()

    # run the NCA for  num_iterations (number of full grid updates, not separate points)
    model.eval()
    num_iterations = 10
    domain = start_domain

    activation_layers = []
    for _ in range(num_iterations):
        domain = update_grid(domain, model)
        activation_map = visualize_activation(domain, model)
        activation_layers.append(activation_map)

    # plot the activation layers, the final grid, and the target domain for comparison
    fig, axs = plt.subplots(1, num_iterations + 2, figsize=(15, 5))
    for i in range(num_iterations):
        axs[i].imshow(activation_layers[i][:, :, 0], cmap='viridis')
        axs[i].set_title(f'Iteration {i}')

    axs[num_iterations].imshow(domain, cmap='viridis')
    axs[num_iterations].set_title('Final Grid')

    axs[num_iterations + 1].imshow(target_domain, cmap='viridis') 
    axs[num_iterations + 1].set_title('Target Domain')

    plt.show()
