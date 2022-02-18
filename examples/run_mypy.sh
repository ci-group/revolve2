#!/bin/sh

cd "$(dirname "$0")"
echo "examples/optimize_modular:" && \
mypy optimize_modular && \
echo "examples/simple_optimization:" && \
mypy simple_optimization && \
echo "examples/simulate_isaac:" && \
mypy simulate_isaac
