"""Installation script."""

import os.path
import pathlib

from setuptools import find_namespace_packages, setup

revolve2_path = pathlib.Path(__file__).parent.parent.parent.resolve()

setup(
    name="revolve2-genotypes-cppnwin",
    version="0.3.8-beta1",
    description="CPPNWIN genotype for modular robots from Revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={"revolve2.genotypes.cppnwin": ["py.typed"]},
    install_requires=[
        f"revolve2-core @ file://{os.path.join(revolve2_path, 'core')}",
        "multineat @ git+https://github.com/ci-group/MultiNEAT.git@v0.10",
    ],
    extras_require={"dev": []},
    zip_safe=False,
)
