#!/bin/sh

cd "$(dirname "$0")"

isort --profile black ../..
