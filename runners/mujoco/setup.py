"""Installation script."""

import os.path
import pathlib

from setuptools import find_namespace_packages, setup

revolve2_path = pathlib.Path(__file__).parent.parent.parent.resolve()

setup(
    name="revolve2-runners-mujoco",
    version="0.3.8-beta1",
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
        "mujoco-python-viewer @ git+https://github.com/rohanpsingh/mujoco-python-viewer@d61433f3991294da455751659925839452b9597e",
        "dm-control>=1.0.3",
        "opencv-python>=4.6.0.66",
    ],
    extras_require={"dev": []},
    zip_safe=False,
)
