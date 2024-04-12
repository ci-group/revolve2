#!/bin/sh

set -e

cd "$(dirname "$0")"

echo "$1:"
mypy --config-file ./mypy.ini "../../$1"
