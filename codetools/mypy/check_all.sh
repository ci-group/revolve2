#!/bin/bash

set -e

cd "$(dirname "$0")"

temp_file=$(mktemp)
../read_project_parts.sh > "$temp_file"

while read -r package; do
    echo "$package:"
    mypy --config-file ./mypy.ini "../../$package"
done < "$temp_file"

rm "$temp_file"
