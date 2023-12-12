#!/bin/sh

cd "$(dirname "$0")"

packages=$(../read_project_parts.sh)

cd ../..

darglint -s sphinx $packages
