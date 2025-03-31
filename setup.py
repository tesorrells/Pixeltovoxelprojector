from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import pybind11
import sys
import os

# Custom build_ext to add compiler-specific flags
class BuildExt(build_ext):
    c_opts = {
        'msvc': ['/O2', '/openmp'],  # Enable optimization and OpenMP for MSVC
        'unix': ['-O3', '-fopenmp'],
    }
    l_opts = {
        'msvc': [],
        'unix': ['-fopenmp'],
    }

    def build_extensions(self):
        ct = self.compiler.compiler_type
        opts = self.c_opts.get(ct, [])
        link_opts = self.l_opts.get(ct, [])
        for ext in self.extensions:
            ext.extra_compile_args = opts
            ext.extra_link_args = link_opts
        build_ext.build_extensions(self)

ext_modules = [
    Extension(
        'process_image_cpp',
        ['process_image.cpp'],
        include_dirs=[
            pybind11.get_include(),  # Use pybind11's function to get the include path
        ],
        language='c++'
    ),
]

setup(
    name='process_image_cpp',
    version='0.0.1',
    ext_modules=ext_modules,
    cmdclass={'build_ext': BuildExt},
)
