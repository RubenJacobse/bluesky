from distutils.core import setup, Extension
from Cython.Build import cythonize

import numpy as np

setup(
    ext_modules=cythonize("area.pyx",
                          annotate=True),
    include_dirs=[np.get_include()]
)
