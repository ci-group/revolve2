#!/bin/sh

set -e

cd "$(dirname "$0")"

../../actor_controller/run_mypy.sh
../../ci_group/run_mypy.sh
../../examples/run_mypy.sh
../../experimentation/run_mypy.sh
../../modular_robot/run_mypy.sh
../../rpi_controller/run_mypy.sh
../../rpi_controller_remote/run_mypy.sh
../../serialization/run_mypy.sh
../../simulation/run_mypy.sh
../../simulators/mujoco/run_mypy.sh
