#!/bin/sh

cd "$(dirname "$0")"
echo "runners/mujoco:"
mypy -p revolve2
