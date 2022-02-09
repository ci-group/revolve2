from setuptools import find_namespace_packages, setup

setup(
    name="revolve2-serialization",
    version="0.0.0",
    description="Serialization classes and helper functions for revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={
        "revolve2.serialization": ["py.typed"],
    },
    install_requires=[],
    extras_require={"dev": []},
    zip_safe=False,
)
