"""Installation script."""

import os.path
import pathlib

from setuptools import find_namespace_packages, setup

revolve2_path = pathlib.Path(__file__).parent.parent.resolve()

setup(
    name="revolve2-rpi-controller",
    version="0.3.8-beta1",
    description="Raspberry Pi controller for revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    install_requires=[
        f"revolve2-actor-controller @ file://{os.path.join(revolve2_path, 'actor_controller')}",
        "pigpio==1.78",
        "jsonschema==4.4.0",
        "adafruit-circuitpython-servokit==1.3.10",
    ],
    extras_require={"dev": ["types-jsonschema==4.4.1"]},
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "revolve2_rpi_controller=revolve2.bin.rpi_controller.revolve2_rpi_controller:main",
        ]
    },
)
