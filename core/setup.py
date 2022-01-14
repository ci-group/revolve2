from setuptools import find_namespace_packages, setup

setup(
    name="revolve2-core",
    version="0.0.0",
    description="Core library for revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={"revolve2": ["py.typed"]},
    install_requires=[
        "networkx>=2.6.3",
        "numpy>=1.21.2",
        "rootpath>=0.1.1",
        "matplotlib>=3.4.3",
        "scipy>=1.7.1",
        "pyrr>=0.10.3",
        "sqlalchemy>=1.4.28",
    ],
    extras_require={
        "dev": ["sqlalchemy-stubs>=0.4", "mypy==0.921", "sphinx-rtd-theme=1.0.0"]
    },
    zip_safe=False,
)
