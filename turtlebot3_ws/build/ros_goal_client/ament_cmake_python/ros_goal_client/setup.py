from setuptools import find_packages
from setuptools import setup

setup(
    name='ros_goal_client',
    version='0.0.0',
    packages=find_packages(
        include=('ros_goal_client', 'ros_goal_client.*')),
)
