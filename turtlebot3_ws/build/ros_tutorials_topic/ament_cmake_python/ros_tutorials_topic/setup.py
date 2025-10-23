from setuptools import find_packages
from setuptools import setup

setup(
    name='ros_tutorials_topic',
    version='0.1.0',
    packages=find_packages(
        include=('ros_tutorials_topic', 'ros_tutorials_topic.*')),
)
