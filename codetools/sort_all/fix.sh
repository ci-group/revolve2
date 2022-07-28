#!/bin/sh

cd "$(dirname "$0")"
dirs="../../actor_controller ../../core ../../genotypes ../../rpi_controller ../../runners ../../serialization ../../standard_resources"
find $dirs -type f -name '__init__.py' -print0 | xargs -0 sort-all
