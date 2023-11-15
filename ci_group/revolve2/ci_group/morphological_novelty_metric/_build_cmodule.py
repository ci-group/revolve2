import os
from os.path import join

import numpy
from Cython.Build import cythonize
from setuptools import Extension, setup


def build() -> None:
    """
    Build the morphological novelty shared object.

    :raises OSError: If the users OS is not Windows or UNIX-based.
    """
    directory_path = os.path.dirname(os.path.abspath(__file__))

    source = join(directory_path, "_calculate_novelty.pyx")

    include = numpy.get_include()

    match os.name:
        case "nt":  # Windows
            extra_compile_args = [
                "/O2",
                "-UNDEBUG",
            ]
        case "posix":  # UNIX-based systems
            extra_compile_args = [
                "-O3",
                "-Werror",
                "-Wno-unreachable-code-fallthrough",
                "-Wno-deprecated-declarations",
                "-Wno-parentheses-equality",
                "-ffast-math",
                "-UNDEBUG",
            ]
        case _:
            raise OSError(
                f"No build parameter set for operating systems of type {os.name}"
            )

    ext = Extension(
        name="calculate_novelty",
        sources=[source],
        include_dirs=[include],
        define_macros=[("NPY_NO_DEPRECATED_API", "NPY_1_7_API_VERSION")],
        extra_compile_args=extra_compile_args,
    )

    ext_modules = cythonize(
        ext,
        include_path=[include],
        compiler_directives={"binding": True, "language_level": 3},
    )

    setup(
        ext_modules=ext_modules,
        script_args=["build_ext", f"--build-lib={directory_path}"],
    )


if __name__ == "__main__":
    build()
