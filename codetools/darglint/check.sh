#!/bin/sh

cd "$(dirname "$0")"

darglint -s sphinx ../../actor_controller/revolve2 ../../ci_group/revolve2 ../../examples ../../experimentation/revolve2 ../../modular_robot/revolve2 ../../rpi_controller/revolve2 ../../rpi_controller_remote/revolve2 ../../serialization/revolve2 ../../simulation/revolve2 ../../simulators/mujoco/revolve2
