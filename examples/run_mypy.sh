#!/bin/sh

cd "$(dirname "$0")"
echo "examples/evaluate_multiple_isolated_robots:" && \
mypy evaluate_multiple_isolated_robots && \
echo "examples/evaluate_single_robot:" && \
mypy evaluate_single_robot && \
echo "examples/experiment_setup:" && \
mypy experiment_setup && \
echo "examples/robot_bodybrain_ea:" && \
mypy robot_bodybrain_ea && \
echo "examples/robot_bodybrain_ea_database:" && \
mypy robot_bodybrain_ea_database && \
echo "examples/robot_brain_cmaes:" && \
mypy robot_brain_cmaes && \
echo "examples/robot_brain_cmaes_database:" && \
mypy robot_brain_cmaes_database && \
echo "examples/rpi_controller_remote:" && \
mypy rpi_controller_remote && \
echo "examples/simple_ea_xor:" && \
mypy simple_ea_xor && \
echo "examples/simple_ea_xor_database:" && \
mypy simple_ea_xor_database && \
echo "examples/simulate_single_robot:" && \
mypy simulate_single_robot
