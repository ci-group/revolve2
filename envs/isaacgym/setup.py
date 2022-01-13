from setuptools import find_namespace_packages, setup

print("ASDASIOKKUDUHIAOUSHDIUASHDIUHAISUHD")
print(find_namespace_packages())

setup(
    name="revolve2-envs-isaacgym",
    version="0.0.0",
    description="Isaac Gym environments for Revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={"revolve2.envs.isaacgym": ["py.typed"]},
    install_requires=["revolve2-core", "isaacgym==1.0rc2"],
    zip_safe=False,
)
