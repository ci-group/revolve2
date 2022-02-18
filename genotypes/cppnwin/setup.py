from setuptools import find_namespace_packages, setup
import pathlib
import os.path

revolve2_path = pathlib.Path(__file__).parent.parent.parent.resolve()

setup(
    name="revolve2-genotypes-cppnwin",
    version="0.1.0-alpha2",
    description="CPPNWIN genotype for modular robots from Revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={"revolve2.genotypes.cppnwin": ["py.typed"]},
    install_requires=[
        f"revolve2-core @ file://{os.path.join(revolve2_path, 'core')}",
        "multineat @ git+https://github.com/ci-group/MultiNEAT.git@f92a347c6efd996b158550ebcab65b965287ecf9"
    ],
    extras_require={"dev": []},
    zip_safe=False,
)
