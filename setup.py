"""
Setup file for the Project Z ROS2 Python package.

This package implements PRM-based path planning and navigation for a planar
robot with obstacle avoidance in simulation.
"""

from setuptools import find_packages, setup
from glob import glob

package_name = 'projectz'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS2 package resource index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Package XML and launch/params files
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/params', glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dev Goti',
    maintainer_email='devgoti1683@gmail.com',
    description=(
        'Project Z: PRM-based path planning and navigation for a planar robot '
        'with obstacle avoidance in simulation.'
    ),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'prm_navigator = projectz.navigator:main'
        ],
    },
)
