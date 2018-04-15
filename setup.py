#!/usr/bin/env python

from setuptools import setup, find_packages

setup(name='opendrive2lanelets',
      version='1.0',
      description='Parser and converter from OpenDRIVE to lanelets',
      author='Stefan Urban',
      author_email='stefan.urban@tum.de',
      url='https://gitlab.lrz.de/koschi/converter',
      packages=find_packages(),
      install_requires=[
          "numpy",
          "lxml",
          "scipy",
      ]
     )
