import os
import shutil
from distutils.command.build_ext import build_ext
from distutils.core import Distribution, Extension
from os.path import join

import numpy
from Cython.Build import cythonize


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

    distribution = Distribution(
        {"name": "morphological_novelty_metric", "ext_modules": ext_modules}
    )
    distribution.package_dir = "morphological_novelty_metric"

    cmd = build_ext(distribution)
    cmd.ensure_finalized()
    cmd.run()

    for output in cmd.get_outputs():  # type: ignore # is an untyped function so need s to be ignored
        # Copy built extensions back to the project
        relative_extension = os.path.relpath(output, cmd.build_lib)
        extension_file_in_library = join(directory_path, relative_extension)
        shutil.copyfile(output, extension_file_in_library)

        # For the built extension, keep original permissions but add read and execute permissions for everyone.
        current_mode = os.stat(extension_file_in_library).st_mode
        os.chmod(extension_file_in_library, current_mode | 0o0555)


if __name__ == "__main__":
    build()
