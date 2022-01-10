#!/bin/sh

cd "$(dirname "$0")"
../core/run_mypy.sh
../genotypes/cppnneat/run_mypy.sh
../envs/isaacgym/run_mypy.sh
