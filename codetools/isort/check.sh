#!/bin/sh

cd "$(dirname "$0")"

packages=$(tr '\n' ' ' < ../packages.txt)

cd ../..

isort --check-only --diff --profile black $packages
