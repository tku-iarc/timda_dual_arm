#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages = ['dual_arm_flexbe_states'],
    package_dir = {'': 'src'}
)

setup(**d)
