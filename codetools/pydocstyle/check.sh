#!/bin/sh

cd "$(dirname "$0")"

packages=$(tr '\n' ' ' < ../../packages.txt)

cd ../..

pydocstyle $packages
