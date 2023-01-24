"""Installation script."""

import os.path
import pathlib

from setuptools import find_namespace_packages, setup

revolve2_path = pathlib.Path(__file__).parent.parent.resolve()

setup(
    name="revolve2-standard-resources",
    version="0.3.8-beta1",
    description="Standard resources such as environments and robots that can be used for testing and research.",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={"revolve2.standard_resources": ["py.typed"]},
    install_requires=[
        f"revolve2-core @ file://{os.path.join(revolve2_path, 'core')}",
        "noise==1.2.2",
    ],
    extras_require={"dev": []},
    zip_safe=False,
)
