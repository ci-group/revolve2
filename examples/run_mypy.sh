#!/bin/sh

cd "$(dirname "$0")"
echo "examples/optimize_modular:" && \
mypy optimize_modular && \
echo "examples/simple_optimization:" && \
mypy simple_optimization && \
echo "examples/simulate_isaac:" && \
mypy simulate_isaac && \
echo "examples/simulate_mujoco:" && \
mypy simulate_mujoco && \
echo "examples/openai_es:" && \
mypy openai_es && \
echo "examples/rpi_controller_remote:" && \
mypy rpi_controller_remote
echo "examples/xor_ga:" && \
mypy xor_ga
echo "examples/xor_de:" && \
mypy xor_de
