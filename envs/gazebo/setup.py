from setuptools import find_namespace_packages, setup

setup(
    name="revolve2-envs-gazebo",
    version="0.0.0",
    description="Gazebo environments for Revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={"revolve2.envs.gazebo": ["py.typed"]},
    install_requires=[
        "revolve2-core",
        "pygazebo @ git+https://github.com/ci-group/pygazebo.git@master#egg=pygazebo",
    ],
    zip_safe=False,
)
