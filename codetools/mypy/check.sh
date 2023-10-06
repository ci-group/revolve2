#!/bin/sh

set -e

cd "$(dirname "$0")"

while read -r package; do
    echo "$package:"
    mypy --config-file ./mypy.ini "../../$package"
done < ../packages.txt
