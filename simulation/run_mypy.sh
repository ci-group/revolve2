#!/bin/sh

cd "$(dirname "$0")"
echo "simulation:"
mypy -p revolve2
