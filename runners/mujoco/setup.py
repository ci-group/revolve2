"""Installation script."""

import os.path
import pathlib

from setuptools import find_namespace_packages, setup

revolve2_path = pathlib.Path(__file__).parent.parent.parent.resolve()

setup(
    name="revolve2-runners-mujoco",
    version="0.3.10-beta1",
    description="Mujoco runner for Revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={
        "revolve2.runners.mujoco": ["py.typed"],
    },
    install_requires=[
        f"revolve2-core @ file://{os.path.join(revolve2_path, 'core')}",
        "mujoco>=2.2.0",
        "mujoco-python-viewer>=0.1.2",
        "dm-control>=1.0.3",
        "opencv-python>=4.6.0.66",
        "pyyaml>=6.0",  # this dependency is only here because it is missing from the mujoco-python-viewer package.
    ],
    extras_require={"dev": []},
    zip_safe=False,
)
