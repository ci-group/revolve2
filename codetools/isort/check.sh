#!/bin/sh

cd "$(dirname "$0")"

isort --check-only --diff --profile black ../../actor_controller ../../core ../../examples ../../genotypes ../../rpi_controller ../../runners ../../serialization ../../standard_resources
