#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['utils']
d['package_dir'] = {'': 'src', 'utils': '../../../Documents/DTmini_grasp/pipeline/utils'}

setup(**d)
