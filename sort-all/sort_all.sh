#!/bin/sh

dirs="./actor_controller ./core ./genotypes ./rpi_controller ./runners ./serialization"
find $dirs -type f -name '__init__.py' -print0 | xargs -0 sort-all 