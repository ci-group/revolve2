#!/bin/sh

cd "$(dirname "$0")"

black ../../ci_group/revolve2 ../../examples ../../experimentation/revolve2 ../../modular_robot/revolve2 ../../modular_robot_simulation/revolve2 ../../simulation/revolve2 ../../simulators/mujoco/revolve2
