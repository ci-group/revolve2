#!/bin/sh

cd "$(dirname "$0")"
echo "standard_resources:"
mypy -p revolve2
