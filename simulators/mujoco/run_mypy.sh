#!/bin/sh

cd "$(dirname "$0")"
echo "simulators/mujoco:"
mypy -p revolve2
