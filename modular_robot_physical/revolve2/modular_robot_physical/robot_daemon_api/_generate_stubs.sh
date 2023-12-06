#!/bin/sh

# To use this tool you have to manually install the capnp-stub-generator package.
# See pyproject.toml.

cd "$(dirname "$0")"

capnp-stub-generator -p "./robot_daemon_protocol.capnp" -c "./robot_daemon_protocol.pyi"
