#!/bin/sh

cd "$(dirname "$0")"
echo "examples/robot_bodybrain_ea:" && \
mypy robot_bodybrain_ea && \
echo "examples/robot_brain_cmaes:" && \
mypy robot_brain_cmaes && \
echo "examples/rpi_controller_remote:" && \
mypy rpi_controller_remote && \
echo "examples/simulate_isaac:" && \
echo "isaacgym not open source. skipping.." && \
echo "examples/simulate_mujoco:" && \
mypy simulate_mujoco && \
echo "examples/xor_de:" && \
mypy xor_de && \
echo "examples/xor_ea:" && \
mypy xor_ea
