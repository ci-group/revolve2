#!/bin/sh

cd "$(dirname "$0")"
echo "rpi_controller:"
mypy -p revolve2
