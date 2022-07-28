#!/bin/sh

set -e

cd "$(dirname "$0")"

../../serialization/run_mypy.sh
../../actor_controller/run_mypy.sh
../../rpi_controller/run_mypy.sh
../../core/run_mypy.sh
../../genotypes/cppnwin/run_mypy.sh
../../runners/mujoco/run_mypy.sh
../../examples/run_mypy_opensource.sh
../../standard_resources/run_mypy.sh
