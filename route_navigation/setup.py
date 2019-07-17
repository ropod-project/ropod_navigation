#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_setup = generate_distutils_setup(
    packages=['route_navigation'],
    package_dir={'route_navigation': 'ros/src/route_navigation'}
)

setup(**package_setup)
