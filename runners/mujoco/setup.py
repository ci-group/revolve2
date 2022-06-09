import os.path
import pathlib

from setuptools import find_namespace_packages, setup

revolve2_path = pathlib.Path(__file__).parent.parent.parent.resolve()

setup(
    name="revolve2-runners-mujoco",
    version="0.2.3-alpha3",
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
        "mujoco-python-viewer>=0.0.3",
    ],
    extras_require={"dev": []},
    zip_safe=False,
)
