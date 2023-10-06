#!/bin/sh

set -e

cd "$(dirname "$0")"

packages="ci_group/revolve2 examples/custom_brain examples/custom_terrain examples/evaluate_multiple_interacting_robots examples/evaluate_multiple_isolated_robots examples/evaluate_single_robot examples/experiment_setup examples/robot_bodybrain_ea examples/robot_bodybrain_ea_database examples/robot_brain_cmaes examples/robot_brain_cmaes_database examples/simple_ea_xor examples/simple_ea_xor_database examples/simulate_single_robot experimentation/revolve2 modular_robot/revolve2 modular_robot_simulation/revolve2 simulation/revolve2 simulators/mujoco/revolve2"

for package in $packages; do
    echo "$package:"
    mypy --config-file ./mypy.ini "../../$package"
done
