from setuptools import find_namespace_packages, setup

setup(
    name="revolve2-genotype-cppnneat",
    version="0.0.0",
    description="CPPN+NEAT genotype for modular robots from Revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={"revolve2.genotypes.cppnneat": ["py.typed"]},
    install_requires=["revolve2-core", "multineat"],
    zip_safe=False,
)
