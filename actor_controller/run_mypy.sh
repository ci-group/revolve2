#!/bin/sh

cd "$(dirname "$0")"
echo "actor_controller:"
mypy -p revolve2
