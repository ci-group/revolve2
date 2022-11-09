#!/bin/sh

# See https://github.com/pypa/pip/issues/10216
# for why it is at the time of writing not possible to create a dev_requirements.txt

cd "$(dirname "$0")"

pip install -e ./runners/isaacgym[dev] && \
pip install -e ./runners/mujoco[dev] && \
pip install -e ./genotypes/cppnwin[dev] && \
pip install -e ./standard_resources[dev] && \
pip install -e ./core[dev] && \
pip install -e ./rpi_controller[dev] && \
pip install -e ./actor_controller[dev] && \
pip install -e ./serialization[dev] && \
pip install -r ./codetools/requirements.txt
pip install -r ./docs/requirements.txt
