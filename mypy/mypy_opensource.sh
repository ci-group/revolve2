#!/bin/sh

cd "$(dirname "$0")"
../serialization/run_mypy.sh && ../actor_controller/run_mypy.sh && ../rpi_controller/run_mypy.sh && ../core/run_mypy.sh && ../genotypes/cppnwin/run_mypy.sh
