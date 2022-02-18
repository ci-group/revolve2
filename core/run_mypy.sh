#!/bin/sh

cd "$(dirname "$0")"
echo "core:"
mypy -p revolve2
