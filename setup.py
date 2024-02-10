#!/usr/bin/env python

"""The setup script."""
import os
from setuptools import setup, find_packages

THIS_DIR = os.path.abspath(os.path.dirname(__file__))

VERSIONFILE = os.path.join(THIS_DIR, "ros2_msg_transform", "__init__.py")
VERSION = None
for line in open(VERSIONFILE, "r").readlines():
    if line.startswith("__version__"):
        VERSION = line.split('"')[1]

with open('README.md') as readme_file:
    README = readme_file.read()

REQUIREMENTS = []

with open("requirements.txt") as f:
    REQUIREMENTS = f.read().splitlines()

setup(
    author="Konstantinos Panayiotou",
    author_email='klpanagi@ece.auth.gr',
    python_requires='>=3.7',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Natural Language :: English',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
    ],
    description="Library to transform  messages to Python structures",
    install_requires=REQUIREMENTS,
    license="MIT license",
    long_description=README,
    include_package_data=True,
    keywords=['ros', 'iot', 'cps'],
    name='ros_msg_transform',
    packages=find_packages(include=['ros_msg_transform', 'ros_msg_transform.*']),
    url='https://github.com/robotics4all/ros-msg-transform',
    version=VERSION,
    zip_safe=False,
)

