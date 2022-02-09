from setuptools import find_namespace_packages, setup
import pathlib
import os.path

revolve2_path = pathlib.Path(__file__).parent.parent.resolve()

setup(
    name="revolve2-namespace",
    version="0.0.0",
    description="Namespace for revolve2. Does nothing but install 'py.typed'.",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={"revolve2": ["py.typed"]},
    install_requires=[],
    extras_require={"dev": []},
    zip_safe=False,
)
