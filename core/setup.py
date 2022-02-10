from setuptools import find_namespace_packages, setup
import pathlib
import os.path

revolve2_path = pathlib.Path(__file__).parent.parent.resolve()

setup(
    name="revolve2-core",
    version="0.0.0",
    description="Core package for revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    install_requires=[
        f"revolve2-actor-controller @ file://{os.path.join(revolve2_path, 'actor_controller')}",
        "numpy>=1.21.2",
        "rootpath>=0.1.1",
        "matplotlib>=3.4.3",
        "scipy>=1.7.1",
        "pyrr>=0.10.3",
        "sqlalchemy>=1.4.28",
    ],
    extras_require={
        "dev": [
            "sqlalchemy-stubs>=0.4",
        ]
    },
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "revolve2.core.modular_robot.physical.create_physical_robot=revolve2.core.modular_robot.physical.create_physical_robot:main",
            "revolve2.analysis.core.plot_ea_fitness=revolve2.analysis.core.plot_ea_fitness:main",
        ]
    },
)
