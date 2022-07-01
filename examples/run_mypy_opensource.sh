#!/bin/sh

cd "$(dirname "$0")"
echo "examples/optimize_modular:" && \
echo "isaacgym not open source. skipping.." && \
echo "examples/simple_optimization:" && \
mypy simple_optimization && \
echo "examples/simulate_isaac:" && \
echo "isaacgym not open source. skipping.." && \
echo "examples/simulate_mujoco:" && \
mypy simulate_mujoco && \
echo "examples/openai_es:" && \
echo "isaacgym not open source. skipping.." && \
echo "examples/rpi_controller_remote:" && \
mypy rpi_controller_remote