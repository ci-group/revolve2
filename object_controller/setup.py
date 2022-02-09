from setuptools import find_namespace_packages, setup
import pathlib
import os.path

revolve2_path = pathlib.Path(__file__).parent.parent.resolve()

setup(
    name="revolve2-object-controller",
    version="0.0.0",
    description="Inidividual object controller for revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={
        "revolve2.object_controller": ["py.typed"],
    },
    install_requires=[
        f"revolve2-serialization @ file://{os.path.join(revolve2_path, 'serialization')}",
        "numpy==1.22.0",
    ],
    extras_require={"dev": []},
    zip_safe=False,
)
