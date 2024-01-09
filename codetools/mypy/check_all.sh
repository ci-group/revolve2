#!/bin/bash

set -e

cd "$(dirname "$0")"

while read -r package; do
    echo "$package:"
    mypy --config-file ./mypy.ini "../../$package" --no-incremental
done < <(../read_project_parts.sh)
