from setuptools import find_namespace_packages, setup

setup(
    name="revolve2-rpi-controller",
    version="0.0.0",
    description="Raspberry Pi controller for revolve2",
    author="Computational Intelligence Group Vrije Universiteit",
    url="https://github.com/ci-group/revolve2",
    packages=find_namespace_packages(),
    package_data={"revolve2/rpi_controller": ["py.typed"]},
    install_requires=[
        "revolve2-object-controller",
        "pigpio==1.78",
    ],
    extras_require={"dev": []},
    zip_safe=False,
    entry_points={
        "console_scripts": [
            "revolve2_rpi_controller=revolve2.rpi_controller.bin.revolve2_rpi_controller:main",
        ]
    },
)
