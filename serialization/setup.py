"""Installation script."""

import pathlib

from setuptools import find_namespace_packages, setup

revolve2_path = pathlib.Path(__file__).parent.parent.resolve()

setup(
    name="revolve2-serialization",
    version="0.3.8-beta1",
    description="Serialization classes and helper functions for revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={"revolve2.serialization": ["py.typed"]},
    install_requires=["typing_extensions==4.1.1"],
    extras_require={"dev": []},
    zip_safe=False,
)
