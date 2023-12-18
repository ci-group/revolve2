#!/bin/sh

cd "$(dirname "$0")"

packages=$(../read_project_parts.sh)

cd ../..

black --diff --check $packages
