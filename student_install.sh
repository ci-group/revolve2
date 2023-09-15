#!/bin/sh

# See https://github.com/pypa/pip/issues/10216
# for why it is at the time of writing not possible to create a student_requirements.txt

cd "$(dirname "$0")"

pip install -e ./ci_group[dev] && \
pip install -e ./simulators/mujoco[dev] && \
pip install -e ./experimentation[dev] && \
pip install -e ./rpi_controller_remote[dev] && \
pip install -e ./rpi_controller[dev] && \
pip install -e ./modular_robot[dev] && \
pip install -e ./simulation[dev] && \
pip install -e ./actor_controller[dev] && \
pip install -e ./serialization[dev] && \
pip install -r ./examples/robot_bodybrain_ea_database/requirements.txt && \
pip install -r ./examples/robot_brain_cmaes_database/requirements.txt && \
pip install -r ./examples/simple_ea_xor_database/requirements.txt
