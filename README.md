
<img align="right" width="150" height="150"  src="./docs/source/logo_light.png">

# Revolve2

Revolve2 is a collection of Python packages used for researching evolutionary algorithms and modular robotics.
Its primary features are a modular robot framework, an abstraction layer around physics simulators, and evolutionary algorithms.

**Documentation: [ci-group.github.io/revolve2](https://ci-group.github.io/revolve2)**

**Installation: [ci-group.github.io/revolve2/installation](https://ci-group.github.io/revolve2/installation)**

**Get help: [github.com/ci-group/revolve2/discussions/categories/ask-for-help](https://github.com/ci-group/revolve2/discussions/categories/ask-for-help)**

[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.8355869.svg)](https://doi.org/10.5281/zenodo.8355869) [![License: LGPL v3](https://img.shields.io/badge/License-LGPL_v3-blue.svg)](./LICENSE) [![CI](https://github.com/ci-group/revolve2/actions/workflows/main.yml/badge.svg)](https://github.com/ci-group/revolve2/actions)

## Sample
Here we create and simulate a modular robot, and then calculate how far it moved over the xy plane. This is a shortened version of `examples/evaluate_single_robot`.
```python
# (...) Omitted preamble

# Create a modular robot.
body = modular_robots_v1.gecko_v1()
brain = BrainCpgNetworkNeighborRandom(body=body, rng=rng)
robot = ModularRobot(body, brain)

# Create a scene.
scene = ModularRobotScene(terrain=terrains.flat())
scene.add_robot(robot)

# Create a simulator.
simulator = LocalSimulator(headless=False)

# Simulate the scene and obtain states sampled during the simulation.
scene_states = simulate_scenes(
    simulator=simulator,
    batch_parameters=make_standard_batch_parameters(),
    scenes=scene,
)

# Get the state at the beginning and end of the simulation.
scene_state_begin = scene_states[0]
scene_state_end = scene_states[-1]

# Retrieve the states of the modular robot.
robot_state_begin = scene_state_begin.get_modular_robot_simulation_state(robot)
robot_state_end = scene_state_end.get_modular_robot_simulation_state(robot)

# Calculate the xy displacement of the robot.
xy_displacement = fitness_functions.xy_displacement(
    robot_state_begin, robot_state_end
)
```

## Furthermore you can beta test the new GUI by running:
`cd gui/viewer/` 

`python main_window.py`
