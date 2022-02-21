#!/bin/sh

cd "$(dirname "$0")"
echo "runners/isaacgym:"
mypy -p revolve2
