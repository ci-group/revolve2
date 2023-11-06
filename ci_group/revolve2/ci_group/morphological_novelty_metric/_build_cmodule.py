import os
import shutil
from distutils.command.build_ext import build_ext
from distutils.core import Distribution, Extension
from os.path import join

import numpy
from Cython.Build import cythonize


def build() -> None:
    """Build the morphological novelty shared object."""
    directory_path = os.path.dirname(os.path.abspath(__file__))

    source = join(directory_path, "_calculate_novelty.pyx")
    include = numpy.get_include()

    if os.name == "nt":  # Windows
        extra_compile_args = [
            "/O2",
        ]
    else:  # UNIX-based systems
        extra_compile_args = [
            "-O3",
            "-Werror",
            "-Wno-unreachable-code-fallthrough",
            "-Wno-deprecated-declarations",
            "-Wno-parentheses-equality",
            "-ffast-math",
        ]
    extra_compile_args.append("-UNDEBUG")  # Cython disables asserts by default.

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

    distribution = Distribution({"name": "extended", "ext_modules": ext_modules})
    distribution.package_dir = "extended"

    cmd = build_ext(distribution)
    cmd.ensure_finalized()
    cmd.run()

    # Copy built extensions back to the project
    for output in cmd.get_outputs():  # type: ignore
        relative_extension = os.path.relpath(output, cmd.build_lib)
        shutil.copyfile(output, relative_extension)
        new_re = f"{directory_path}/calculate_novelty.so"
        shutil.copyfile(relative_extension, new_re)
        os.remove(relative_extension)
        mode = os.stat(new_re).st_mode
        mode |= (mode & 0o444) >> 2
        os.chmod(new_re, mode)


if __name__ == "__main__":
    build()
