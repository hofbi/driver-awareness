#!/usr/bin/env python
"""Setup file for awareness detector python module"""

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

SETUP_ARGS = generate_distutils_setup(
    packages=["awareness_detector"], package_dir={"": "src"}
)

setup(**SETUP_ARGS)
