#!/bin/sh

cd "$(dirname "$0")"

darglint -s sphinx ../../actor_controller/revolve2 ../../core/revolve2 ../../examples ../../genotypes/cppnwin/revolve2 ../../rpi_controller/revolve2 ../../runners/mujoco/revolve2 ../../serialization/revolve2 ../../standard_resources/revolve2
