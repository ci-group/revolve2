#!/bin/sh

cd "$(dirname "$0")"

pydocstyle ../../actor_controller ../../core ../../examples ../../genotypes ../../rpi_controller ../../runners ../../serialization ../../standard_resources
