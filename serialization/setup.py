from setuptools import find_namespace_packages, setup
import pathlib
import os.path

revolve2_path = pathlib.Path(__file__).parent.parent.resolve()

setup(
    name="revolve2-serialization",
    version="0.0.0",
    description="Serialization classes and helper functions for revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    install_requires=[
        f"revolve2-namespace @ file://{os.path.join(revolve2_path, 'namespace')}",
    ],
    extras_require={"dev": []},
    zip_safe=False,
)
