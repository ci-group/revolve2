#!/bin/sh

cd "$(dirname "$0")"

capnp-stub-generator -p "./robot_daemon_protocol.capnp" -c "./robot_daemon_protocol.pyi"
