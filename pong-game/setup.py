#!/usr/bin/python

# Copyright (c) 2008-2016, Rethink Robotics, Inc.
# All rights reserved.

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['PongGameLib']
d['package_dir'] = {'': 'src'}

setup(**d)
