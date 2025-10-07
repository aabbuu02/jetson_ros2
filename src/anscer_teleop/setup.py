"""
@file setup.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

from setuptools import setup
import os
from glob import glob

package_name = 'anscer_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abubakarsiddiq',
    maintainer_email='abubakarsiddiq@todo.todo',
    description='Teleop for Anscer robots',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'anscer_teleop_key = anscer_teleop.anscer_teleop_key:main',
        ],
    },
)
