#!/bin/sh

cd "$(dirname "$0")"

darglint -s sphinx ../../actor_controller ../../core ../../examples ../../genotypes ../../rpi_controller ../../runners ../../serialization ../../standard_resources
