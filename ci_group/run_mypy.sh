#!/bin/sh

cd "$(dirname "$0")"
echo "ci_group:"
mypy -p revolve2
