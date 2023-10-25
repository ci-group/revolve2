import os
from os.path import join

import numpy as np
from Cython.Build import cythonize
from setuptools import Extension, setup

directory_path = os.path.dirname(os.path.abspath(__file__))

source = join(directory_path, "_calculate_novelty.pyx")
include = np.get_include()

ext = Extension(
    name="calculate_novelty",
    sources=[source],
    include_dirs=[include],
    define_macros=[("NPY_NO_DEPRECATED_API", "NPY_1_7_API_VERSION")],
)

setup(
    name="morphological_novelty",
    author="Oliver Weissl",
    author_email="oliver.weissl@outlook.com",
    ext_modules=cythonize(ext, annotate=True),
)
