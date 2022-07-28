#!/bin/sh

cd "$(dirname "$0")"

pyflakes ../../actor_controller ../../core ../../examples ../../genotypes ../../rpi_controller ../../runners ../../serialization ../../standard_resources
