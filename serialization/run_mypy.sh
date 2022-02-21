#!/bin/sh

cd "$(dirname "$0")"
echo "serialization:"
mypy -p revolve2
