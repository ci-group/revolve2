"""Installation script."""

import os.path
import pathlib

from setuptools import find_namespace_packages, setup

revolve2_path = pathlib.Path(__file__).parent.parent.resolve()

setup(
    name="revolve2-actor-controller",
    version="0.3.8-beta1",
    description="Individual Actor controller for revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={
        "revolve2.actor_controller": ["py.typed"],
        "revolve2.actor_controllers.cpg": ["py.typed"],
    },
    install_requires=[
        f"revolve2-serialization @ file://{os.path.join(revolve2_path, 'serialization')}",
        "numpy>=1.21.6",
    ],
    extras_require={"dev": []},
    zip_safe=False,
)
