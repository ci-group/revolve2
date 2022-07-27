#!/bin/sh

set -e

cd "$(dirname "$0")"

echo "--------------"
echo "mypy"
echo "--------------"
./mypy/check_all.sh

echo "--------------"
echo "pyflakes"
echo "--------------"
./pyflakes/check.sh

echo "--------------"
echo "sort-all"
echo "--------------"
./sort_all/fix.sh

echo "--------------"
echo "black"
echo "--------------"
./black/fix.sh

echo "--------------"
echo "isort"
echo "--------------"
./isort/fix.sh

echo "--------------"
echo "pydocstyle"
echo "--------------"
./pydocstyle/check.sh

echo "--------------"
echo "darglint"
echo "--------------"
./darglint/check.sh
