#!/bin/sh

# See https://github.com/pypa/pip/issues/10216
# for why it is at the time of writing not possible to create a dev_requirements.txt

pip install -e ./runners/isaacgym[dev]
pip install -e ./genotypes/cppnwin[dev]
pip install -e ./core[dev]
pip install -e ./rpi_controller[dev]
pip install -e ./actor_controller[dev]
pip install -e ./serialization[dev]
