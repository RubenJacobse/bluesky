from distutils.core import setup, Extension
from Cython.Build import cythonize
from Cython.Compiler import Options

Options.docstrings = True

import numpy as np

setup(
    ext_modules=cythonize("*.pyx",
                          annotate=True,
                          language_level="3"),
    include_dirs=[np.get_include()]
)
