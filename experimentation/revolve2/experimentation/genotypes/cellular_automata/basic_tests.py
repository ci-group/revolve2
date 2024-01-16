import numpy as np
from develop import develop
from ca_genotype import CAGenotype
from revolve2.modular_robot import MorphologicalMeasures, get_body_states_single_robot
import matplotlib.pyplot as plt
from revolve2.ci_group import fitness_functions, modular_robots, terrains


def main():
    dsize = 7
    domain = np.zeros((dsize, dsize))
    domain[dsize // 2, dsize // 2] = 1

    g = CAGenotype()
    g.set_params(domain, 15)
    g.generate_random_genotype(10)
    g.generate_body()
    g.set_core(dsize // 2, dsize // 2)

    ca_grid = g.ca_grid
    ca_grid[3][2] = 2
    ca_grid[4][5] = 2
    g.ca_grid = ca_grid

    print("genotype: \n", g.get_grid())

    body = develop(g)

    body_measures = MorphologicalMeasures(body=body)
    grid_arr = np.zeros_like(g.ca_grid)
    grid = body_measures.body_as_grid

    for x in range(body_measures.bounding_box_depth):
        for y in range(body_measures.bounding_box_width):
            # if grid[x][y][body_measures.core_grid_position[2]] is not None:
            if "Brick" in str(grid[x][y][body_measures.core_grid_position[2]]):
                grid_arr[x][y] = 1
            elif "ActiveHinge" in str(grid[x][y][body_measures.core_grid_position[2]]):
                grid_arr[x][y] = 2
            elif "Core" in str(grid[x][y][body_measures.core_grid_position[2]]):
                grid_arr[x][y] = 3

    print("body.find_bricks(): \n", body.find_bricks())

    print("body as grid: \n", body_measures.body_as_grid)

    print(grid_arr)

    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(1, 2)
    axs[0].imshow(g.ca_grid)
    axs[1].imshow(grid_arr)
    plt.show()


def additional():
    dsize = 7
    domain = np.zeros((dsize, dsize))
    domain[dsize // 2, dsize // 2] = 1
    rule_set = {}

    g = CAGenotype()
    g.set_params(init_state=domain, iterations=3, rule_set=rule_set)
    g.generate_random_genotype(30)
    g.generate_body()
    g.set_core(dsize // 2, dsize // 2)

    body = develop(g)

    # plt.imshow(g.ca_grid)
    # plt.show()

    body_measures = MorphologicalMeasures(body=body)
    grid_arr = np.zeros_like(g.ca_grid)
    grid = body_measures.body_as_grid
    print("core in ca_grid", g.core_position)
    print("core in body_measures", body_measures.core_grid_position)

    for x in range(body_measures.bounding_box_depth):
        for y in range(body_measures.bounding_box_width):
            # if grid[x][y][body_measures.core_grid_position[2]] is not None:
            if "Brick" in str(grid[x][y][body_measures.core_grid_position[2]]):
                grid_arr[x][y] = 1
            elif "ActiveHinge" in str(grid[x][y][body_measures.core_grid_position[2]]):
                grid_arr[x][y] = 2
            elif "Core" in str(grid[x][y][body_measures.core_grid_position[2]]):
                grid_arr[x][y] = 3

    print(grid)
    fig, axs = plt.subplots(nrows=1, ncols=2)
    axs[0].imshow(grid_arr)
    axs[1].imshow(g.ca_grid)
    cbar = fig.colorbar(axs[1].imshow(g.ca_grid), ax=axs, fraction=0.02)


def modular_robot_test():
    import asyncio
    import numpy as np

    from revolve2.ci_group import terrains
    from revolve2.ci_group.logging import setup_logging
    from revolve2.ci_group.rng import make_rng
    from revolve2.ci_group.simulation import create_batch_single_robot_standard
    from revolve2.modular_robot import (
        ActiveHinge,
        Body,
        Brick,
        ModularRobot,
        RightAngles,
    )
    from revolve2.modular_robot.brains import BrainCpgNetworkNeighborRandom
    from revolve2.simulators.mujoco import LocalRunner
    import revolve2.ci_group.modular_robots as modrob

    dsize = 7
    domain = np.zeros((dsize, dsize))
    domain[dsize // 2, dsize // 2] = 1

    g = CAGenotype()
    g.set_params(init_state=domain, iterations=5, rule_set={})
    g.rule_set = {
        (1.0, 0.0, 0.0, 0.0): 1.0,
        (0.0, 1.0, 0.0, 0.0): 1.0,
        (0.0, 0.0, 0.0, 1.0): 1.0,
    }
    g.generate_body()
    g.set_core(dsize // 2, dsize // 2)

    g.ca_grid[4][3] = 2
    g.ca_grid[3][4] = 2

    body = develop(g)

    body_measures = MorphologicalMeasures(body=body)
    grid_arr = np.zeros_like(g.ca_grid)
    grid = body_measures.body_as_grid
    print("core in ca_grid", g.core_position)
    print("core in body_measures", body_measures.core_grid_position)

    for x in range(body_measures.bounding_box_depth):
        for y in range(body_measures.bounding_box_width):
            # if grid[x][y][body_measures.core_grid_position[2]] is not None:
            if "Brick" in str(grid[x][y][body_measures.core_grid_position[2]]):
                grid_arr[x][y] = 1.0
            elif "ActiveHinge" in str(grid[x][y][body_measures.core_grid_position[2]]):
                grid_arr[x][y] = 2.0
            elif "Core" in str(grid[x][y][body_measures.core_grid_position[2]]):
                grid_arr[x][y] = 3.0

    print(g.ca_grid)

    """
    fig, axs = plt.subplots(nrows=1, ncols=2)
    axs[0].imshow(grid_arr)
    axs[1].imshow(g.ca_grid)
    cbar = fig.colorbar(axs[1].imshow(g.ca_grid), ax=axs, fraction=0.02)
    plt.show()
    """

    """Run the simulation."""
    # Set up a random number generater, used later.
    RNG_SEED = 5
    rng = make_rng(RNG_SEED)

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

    runner = LocalRunner()
    # Here we use the runner to run a batch.
    # Once a runner finishes a batch, it can be reused to run a new batch.
    # (We only run one batch in this tutorial, so we only use the runner once.)
    asyncio.run(runner.run_batch(batch))

    """
    
    # Create a runner that will perform the simulation.
    # This tutorial chooses to use Mujoco, but your version of revolve might contain other simulators as well.
    runner = LocalRunner(headless=True)
    # Here we use the runner to run a batch.
    # Once a runner finishes a batch, it can be reused to run a new batch.
    # (We only run one batch in this tutorial, so we only use the runner once.)

    
    results = asyncio.run(runner.run_batch(batch))
    environment_results = results.environment_results[0]

        # We have to map the simulation results back to robot body space.
    # This function calculates the state of the robot body at the start and end of the simulation.
    body_state_begin, body_state_end = get_body_states_single_robot(
        body, environment_results
    )

    # Calculate the xy displacement from the body states.
    xy_displacement = fitness_functions.xy_displacement(
        body_state_begin, body_state_end
    )

    print(xy_displacement)
    """


if __name__ == "__main__":
    # import random
    # for i in range(32, 33):
    #    random.seed(i)
    #    additional()
    #    print('seed = ', i)
    #    plt.show()

    modular_robot_test()
