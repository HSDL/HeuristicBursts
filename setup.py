# -*- coding: utf-8 -*-

from setuptools import setup, find_packages

with open('README.md') as f:
    readme = f.read()

with open('LICENSE') as f:
    license = f.read()

setup(
    name='heuristic_bursts',
    version='0.0.1',
    long_description=readme,
    author='Christopher McComb',
    author_email='chris.c.mccomb@gmail.com',
    license=license,
    packages=find_packages(exclude=('heuristic_bursts', 'tests', 'wec'))
)