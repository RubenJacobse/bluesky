from distutils.core import setup, Extension
from Cython.Build import cythonize

import numpy as np

setup(
    ext_modules=cythonize("cy_geo.pyx",
                          annotate=True,
                          language_level="3"),
    include_dirs=[np.get_include()]
)
