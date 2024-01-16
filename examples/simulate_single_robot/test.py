"""
Simulate and visualize a single modular robot.

You learn:
- How to create a robot body with a basic controller.
- How to simulate and see a robot.
"""

import asyncio
import numpy as np

from revolve2.ci_group import terrains
from revolve2.ci_group.logging import setup_logging
from revolve2.ci_group.rng import make_rng
from revolve2.ci_group.simulation import create_batch_single_robot_standard
from revolve2.modular_robot import ActiveHinge, Body, Brick, ModularRobot, RightAngles
from revolve2.modular_robot.brains import BrainCpgNetworkNeighborRandom
from revolve2.simulators.mujoco import LocalRunner
import revolve2.ci_group.modular_robots as modrob

from revolve2.modular_robot import MorphologicalMeasures



def make_body() -> Body:
    """
    Create a body for the robot.

    :returns: The created body.
    """
    # Set up standard logging.
    setup_logging()

    # A modular robot body follows a 'tree' structure.
    # The 'Body' class automatically creates a center 'core'.
    # From here, other modular can be attached.
    # Modules can be attach in a rotated fashion.
    # This can be any angle, although the original design takes into account only multiples of 90 degrees.
    body = Body()

    body.core.left = ActiveHinge(0.0)
    body.core.left.attachment = Brick(0.0)
    body.core.left.attachment.front = ActiveHinge(0.0)
    body.core.left.attachment.front.attachment = Brick(0.0)

    body.core.right = ActiveHinge(0.0)
    body.core.right.attachment = Brick(0.0)
    body.core.right.attachment.front = ActiveHinge(0.0)
    body.core.right.attachment.front.attachment = Brick(0.0)

    body.core.front = ActiveHinge(0.0)
    body.core.front.attachment = Brick(0.0)
    body.core.front.attachment.front = ActiveHinge(0.0)
    body.core.front.attachment.front.attachment = Brick(0.0)

    body.core.back = ActiveHinge(0.0)
    body.core.back.attachment = Brick(0.0)
    body.core.back.attachment.front = ActiveHinge(0.0)
    body.core.back.attachment.front.attachment = Brick(0.0)

    # A body needs to be finalized after building.
    # This sets some variables on each module that makes it easier to work with the body later.
    # Don't worry if you forget to the finalize; the framework will raise an error when you attempt to perform an action that required finalization beforehand.
    
    body.finalize()
    return body

def calculate_symmetry(arr, output_measure='all'):
    rows, cols = arr.shape
    min_dim = min(rows, cols)
    center_row = rows // 2
    center_col = cols // 2
    sum_x = 0
    sum_y = 0
    sum_diag1 = 0
    sum_diag2 = 0

    for i in range(rows):
        for j in range(cols):
            # Check for symmetry along the X-axis relative to the central row
            if arr[i][j] == arr[rows - i - 1][j]:
                sum_x += 1
            # Check for symmetry along the Y-axis relative to the central column
            if arr[i][j] == arr[i][cols - j - 1]:
                sum_y += 1
            # Check for symmetry along the diagonal from top-left to bottom-right
            if i < min_dim and j < min_dim and arr[i][j] == arr[rows - j - 1][cols - i - 1]:
                sum_diag1 += 1
            # Check for symmetry along the diagonal from top-right to bottom-left
            if i < min_dim and j < min_dim and arr[i][j] == arr[j][i]:
                sum_diag2 += 1

    Sx = sum_x / (rows * cols)
    Sy = sum_y / (rows * cols)
    S_diag1 = sum_diag1 / ((min_dim * (min_dim + 1)) / 2)  
    S_diag2 = sum_diag2 / ((min_dim * (min_dim + 1)) / 2)  

    if output_measure == 'all':
        return Sx, Sy, S_diag1, S_diag2
    elif output_measure == 'max':
        return max(Sx, Sy, S_diag1, S_diag2)
    elif output_measure == 'mean':
        return np.mean(Sx, Sy, S_diag1, S_diag2)



def main() -> None:
    """Run the simulation."""
    # Set up a random number generater, used later.
    RNG_SEED = 5
    rng = make_rng(RNG_SEED)

    # Create a body for the robot.
    body = make_body()
    # Create a brain for the robot.
    # We choose a 'CPG' brain with random parameters (the exact working will not be explained here).
    brain = BrainCpgNetworkNeighborRandom(rng)
    # Combine the body and brain into a modular robot.
    robot = ModularRobot(body, brain)

    # Create a batch containing the robot in a flat terrain.
    # A batch describes a set of simulations to perform.
    # In this case there is a single simulation containing a single robot.
    # We use the 'standard' parameters for robot simulation.
    # You probably don't have to worry about this, but if you are interested, take a look inside the function.
    
    batch = create_batch_single_robot_standard(robot=robot, terrain=terrains.flat())
    #batch = create_batch_single_robot_standard(robot=robot, terrain=terrains.crater((10, 10), 0.5, 20))

    body_measures = MorphologicalMeasures(body=body)
    grid = body_measures.body_as_grid
    print(grid)

    x_head, y_head, z_head = body_measures.core_grid_position
    grid_arr = np.zeros((body_measures.bounding_box_depth, body_measures.bounding_box_width))
    for x in range(body_measures.bounding_box_depth):
        for y in range(body_measures.bounding_box_width):
            if grid[x][y][body_measures.core_grid_position[2]] is not None:
                grid_arr[x][y] = 1
    grid_arr[x_head][y_head] = 2

    print(grid_arr)
    print(body_measures.core_grid_position)

    print(calculate_symmetry(grid_arr))


    # Create a runner that will perform the simulation.
    # This tutorial chooses to use Mujoco, but your version of revolve might contain other simulators as well.
    runner = LocalRunner()
    # Here we use the runner to run a batch.
    # Once a runner finishes a batch, it can be reused to run a new batch.
    # (We only run one batch in this tutorial, so we only use the runner once.)
    asyncio.run(runner.run_batch(batch))


if __name__ == "__main__":
    main()
