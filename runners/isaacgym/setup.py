"""Installation script."""

import os.path
import pathlib

from setuptools import find_namespace_packages, setup

revolve2_path = pathlib.Path(__file__).parent.parent.parent.resolve()

setup(
    name="revolve2-runners-isaacgym",
    version="0.3.8-beta1",
    description="Isaac Gym runner for Revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={
        "revolve2.runners.isaacgym": ["py.typed"],
        "revolve2.analysis.isaacgym": ["py.typed"],
    },
    install_requires=[
        f"revolve2-core @ file://{os.path.join(revolve2_path, 'core')}",
        "isaacgym==1.0rc4",
        "colored>=1.4.3",
    ],
    extras_require={"dev": []},
    zip_safe=False,
)
