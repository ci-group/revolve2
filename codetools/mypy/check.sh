#!/bin/sh

set -e

cd "$(dirname "$0")"

../../ci_group/run_mypy.sh
../../examples/run_mypy.sh
../../experimentation/run_mypy.sh
../../modular_robot/run_mypy.sh
../../modular_robot_simulation/run_mypy.sh
../../simulation/run_mypy.sh
../../simulators/mujoco/run_mypy.sh
