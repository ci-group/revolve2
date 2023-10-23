"""Installation script."""

import os.path
import pathlib

from setuptools import find_namespace_packages, setup

revolve2_path = pathlib.Path(__file__).parent.parent.resolve()

setup(
    name="revolve2-modular-robot-physical",
    version="0.3.2b2",
    description="Revolve2: Everything for physical modular robot control. This package is intended to be installed on the modular robot's operating system.",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    install_requires=[
        f"revolve2-modular-robot @ file://{os.path.join(revolve2_path, 'actor_controller')}",
    ],
    extras_require={"v1": ["pigpio==1.78"], "v2": [], "all": ["pigpio==1.78"]},
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "revolve2_modular_robot_physical=revolve2.modular_robot_physical._physical_robot_controller:main",
        ]
    },
)