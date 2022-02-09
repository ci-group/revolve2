from setuptools import find_namespace_packages, setup

setup(
    name="revolve2-object-controller",
    version="0.0.0",
    description="Inidividual object controller for revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={
        "revolve2.object_controller": ["py.typed"],
        "revolve2.object_controllers": ["py.typed"],
    },
    install_requires=["revolve2-serialization", "numpy==1.22.0", "jsonschema==4.4.0"],
    extras_require={"dev": ["types-jsonschema==4.4.1"]},
    zip_safe=False,
)
