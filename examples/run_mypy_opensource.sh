#!/bin/sh

cd "$(dirname "$0")"
echo "examples/robot_bodybrain_ea:" && \
mypy robot_bodybrain_ea && \
echo "examples/robot_brain_cmaes:" && \
mypy robot_brain_cmaes && \
echo "examples/rpi_controller_remote:" && \
mypy rpi_controller_remote && \
echo "examples/simulate_single_robot:" && \
mypy simulate_single_robot && \
echo "examples/evaluate_single_robot:" && \
mypy evaluate_single_robot && \
echo "examples/evaluate_multiple_isolated_robots:" && \
mypy evaluate_multiple_isolated_robots && \
echo "examples/xor_de:" && \
mypy xor_de && \
echo "examples/xor_ea:" && \
mypy xor_ea
