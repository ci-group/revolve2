from setuptools import find_namespace_packages, setup
import pathlib
import os.path

revolve2_path = pathlib.Path(__file__).parent.parent.parent.resolve()

setup(
    name="revolve2-genotypes-cppnwin",
    version="0.0.0",
    description="CPPNWIN genotype for modular robots from Revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    install_requires=[
        f"revolve2-core @ file://{os.path.join(revolve2_path, 'core')}",
        "multineat",
    ],
    extras_require={"dev": []},
    zip_safe=False,
)
