#!/bin/sh

# Installs all Revolve2 packages in editable mode as well as all example requirements.
# This is the same as installing `editable_requirements.txt`.

cd "$(dirname "$0")"

pip install -r ./editable_requirements.txt
