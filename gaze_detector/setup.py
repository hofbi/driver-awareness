# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
"""Awareness detector setup.py"""
from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
SETUP_ARGS = generate_distutils_setup(
    packages=["gaze_detector"],
    package_dir={"": "src"},
)

setup(**SETUP_ARGS)
