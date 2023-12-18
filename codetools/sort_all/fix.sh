#!/bin/sh

cd "$(dirname "$0")"

packages=$(../read_project_parts.sh)

cd ../..

find $packages -type f -name '__init__.py' -print0 | xargs -0 sort-all
