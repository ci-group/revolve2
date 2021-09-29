from setuptools import find_packages, setup

setup(
    name="revolve2",
    version="0.0.0",
    description="Evolutionary algorithms framework",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_packages(),
    package_data={"revolve2": ["py.typed"]},
    install_requires=[
        "networkx>=2.6.3",
        "numpy>=1.21.2",
        "rootpath>=0.1.1",
        "matplotlib>=3.4.3",
    ],
)
