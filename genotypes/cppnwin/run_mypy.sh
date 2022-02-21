#!/bin/sh

cd "$(dirname "$0")"
echo "genotypes/cppnwin:"
mypy -p revolve2
